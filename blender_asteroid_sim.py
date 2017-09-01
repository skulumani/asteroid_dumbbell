"""Simulation of controlled dumbbell around Itokawa with 
simulated imagery using Blender

Blender simulation in the asteroid frame

1 September 2017 - Shankar Kulumani
"""
from __future__ import absolute_import, division, print_function, unicode_literals
from scipy import integrate
import numpy as np
import pdb
import h5py, cv2

import visualization.plotting as plotting
from visualization import blender_camera

from dynamics import asteroid, dumbbell, controller, eoms
from kinematics import attitude

from visualization import blender

import inertial_driver as idriver
import relative_driver as rdriver
import datetime


def blender_asteroid_frame_sim(gen_images=False):
    # simulation parameters
    output_path = './visualization/blender'
    asteroid_name = 'itokawa_low'
    # create a HDF5 dataset
    hdf5_path = './data/asteroid_circumnavigate/{}_asteroid_circumnavigate.hdf5'.format(
        datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"))
    dataset_name = 'landing'
    render = 'BLENDER'
    image_modulus = 1
    RelTol = 1e-6
    AbsTol = 1e-6
    ast_name = 'itokawa'
    num_faces = 64
    t0 = 0
    dt = 1
    tf = 3600 * 6
    num_steps = 3600 * 6

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # instantiate the blender scene once
    camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene = blender.blender_init(render_engine=render, asteroid_name=asteroid_name)

    # get some of the camera parameters
    K = blender_camera.get_calibration_matrix_K_from_blender(camera)

    periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
    periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

    # set initial state for inertial EOMs
    # initial_pos = np.array([2.550, 0, 0]) # km for center of mass in body frame
    initial_pos = np.array([3, 0, 0])
    initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
    initial_R = attitude.rot3(np.pi/2).reshape(9) # transforms from dumbbell body frame to the inertial frame
    initial_w = np.array([0.01, 0.01, 0.01])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # instantiate ode object
    # system = integrate.ode(eoms_controlled_blender)
    system = integrate.ode(eoms.eoms_controlled_relative_blender_ode)
    system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1000)
    system.set_initial_value(initial_state, t0)
    system.set_f_params(dum, ast)

    i_state = np.zeros((num_steps+1, 18))
    time = np.zeros(num_steps+1)
    i_state[0, :] = initial_state

    with h5py.File(hdf5_path) as image_data:
        # create a dataset
        if gen_images:
            images = image_data.create_dataset(dataset_name, (244, 537, 3,
                                                              num_steps/image_modulus),
                                               dtype='uint8')
            RT_blender = image_data.create_dataset('RT',
                                                   (num_steps/image_modulus,
                                                    12)) 
            R_i2bcam = image_data.create_dataset('R_i2bcam',
                                                 (num_steps/image_modulus, 9))
        
        ii = 1
        while system.successful() and system.t < tf:
            # integrate the system and save state to an array
            
            time[ii] = (system.t + dt)
            i_state[ii, :] = (system.integrate(system.t + dt))
            # generate the view of the asteroid at this state
            if int(time[ii]) % image_modulus== 0 and gen_images:
                img, RT, R = blender.gen_image(i_state[ii,0:3], i_state[ii,6:15].reshape((3,3)), 
                                ast.omega * (time[ii] - 3600),
                                camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene,
                                [5, 0, 1], 'test')

                images[:, :, :, ii//image_modulus - 1] = img
                RT_blender[ii//image_modulus -1, :] = RT.reshape(12)
                R_i2bcam[ii//image_modulus -1, :] = R.reshape(9)


            # do some image processing and visual odometry
            print(system.t)
            ii += 1

        image_data.create_dataset('K', data=K)
        image_data.create_dataset('i_state', data=i_state)
        image_data.create_dataset('time', data=time)

