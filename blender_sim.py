"""Simulation of controlled dumbbell around Itokawa with 
simulated imagery using Blender

This will generate the imagery of Itokawa from a spacecraft following 
a vertical descent onto the surface.

4 August 2017 - Shankar Kulumani
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

def eoms_controlled_blender(t, state, dum, ast):
    """Inertial dumbbell equations of motion about an asteroid 
    
    This method must be used with the scipy.integrate.ode class instead of the
    more convienent scipy.integrate.odeint. In addition, we can control the
    dumbbell given full state feedback. Blender is used to generate imagery
    during the simulation.

    Inputs:
        t - Current simulation time step
        state - (18,) array which defines the state of the vehicle 
            pos - (3,) position of the dumbbell with respect to the
            asteroid center of mass and expressed in the inertial frame
            vel - (3,) velocity of the dumbbell with respect to the
            asteroid center of mass and expressed in the inertial frame
            R - (9,) attitude of the dumbbell with defines the
            transformation of a vector in the dumbbell frame to the
            inertial frame ang_vel - (3,) angular velocity of the dumbbell
            with respect to the inertial frame and represented in the
            dumbbell frame
        ast - Asteroid class object holding the asteroid gravitational
        model and other useful parameters
    """
    # unpack the state
    pos = state[0:3] # location of the center of mass in the inertial frame
    vel = state[3:6] # vel of com in inertial frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame

    Ra = attitude.rot3(ast.omega*t, 'c') # asteroid body frame to inertial frame

    # unpack parameters for the dumbbell
    J = dum.J

    rho1 = dum.zeta1
    rho2 = dum.zeta2

    # position of each mass in the asteroid frame
    z1 = Ra.T.dot(pos + R.dot(rho1))
    z2 = Ra.T.dot(pos + R.dot(rho2))

    z = Ra.T.dot(pos) # position of COM in asteroid frame

    # compute the potential at this state
    (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
    (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

    F1 = dum.m1*Ra.dot(U1_grad)
    F2 = dum.m2*Ra.dot(U2_grad)

    M1 = dum.m1 * attitude.hat_map(rho1).dot(R.T.dot(Ra).dot(U1_grad))
    M2 = dum.m2 * attitude.hat_map(rho2).dot(R.T.dot(Ra).dot(U2_grad))
    
    # generate image at this current state only at a specifc time
    # blender.driver(pos, R, ast.omega * t, [5, 0, 1], 'test' + str.zfill(str(t), 4))
    # use the imagery to figure out motion and pass to the controller instead
    # of the true state

    # calculate the desired attitude and translational trajectory
    des_att_tuple = controller.body_fixed_pointing_attitude(t, state)
    des_tran_tuple = controller.traverse_then_land_vertically(t, ast, final_pos=[0.550, 0, 0],
                                                              initial_pos=[2.550, 0, 0],
                                                              descent_tf=3600)
    # input trajectory and compute the control inputs
    # compute the control input
    u_m = controller.attitude_controller(t, state, M1+M2, dum, ast, des_att_tuple)
    u_f = controller.translation_controller(t, state, F1+F2, dum, ast, des_tran_tuple)

    pos_dot = vel
    vel_dot = 1/(dum.m1+dum.m2) *(F1 + F2 + u_f)
    R_dot = R.dot(attitude.hat_map(ang_vel)).reshape(9)
    ang_vel_dot = np.linalg.inv(J).dot(-np.cross(ang_vel,J.dot(ang_vel)) + M1 + M2 + u_m)

    statedot = np.hstack((pos_dot, vel_dot, R_dot, ang_vel_dot))

    return statedot


def eoms_controlled_blender_traverse_then_land(t, state, dum, ast):
    """Inertial dumbbell equations of motion about an asteroid 
    
    This method must be used with the scipy.integrate.ode class instead of the
    more convienent scipy.integrate.odeint. In addition, we can control the
    dumbbell given full state feedback. Blender is used to generate imagery
    during the simulation.

    The spacecraft will move horizontally for the first 3600 sec to a positon 
    [2.550, 0, 0] in the asteroid (and inertial) frame, then descend vertically 
    in the asteroid frame.

    Inputs:
        t - Current simulation time step
        state - (18,) array which defines the state of the vehicle 
            pos - (3,) position of the dumbbell with respect to the
            asteroid center of mass and expressed in the inertial frame
            vel - (3,) velocity of the dumbbell with respect to the
            asteroid center of mass and expressed in the inertial frame
            R - (9,) attitude of the dumbbell with defines the
            transformation of a vector in the dumbbell frame to the
            inertial frame ang_vel - (3,) angular velocity of the dumbbell
            with respect to the inertial frame and represented in the
            dumbbell frame
        ast - Asteroid class object holding the asteroid gravitational
        model and other useful parameters
    """
    # unpack the state
    pos = state[0:3] # location of the center of mass in the inertial frame
    vel = state[3:6] # vel of com in inertial frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame

    Ra = attitude.rot3(ast.omega*(t - 3600), 'c') # asteroid body frame to inertial frame

    # unpack parameters for the dumbbell
    J = dum.J

    rho1 = dum.zeta1
    rho2 = dum.zeta2

    # position of each mass in the asteroid frame
    z1 = Ra.T.dot(pos + R.dot(rho1))
    z2 = Ra.T.dot(pos + R.dot(rho2))

    z = Ra.T.dot(pos) # position of COM in asteroid frame

    # compute the potential at this state
    (U1, U1_grad, U1_grad_mat, U1laplace) = ast.polyhedron_potential(z1)
    (U2, U2_grad, U2_grad_mat, U2laplace) = ast.polyhedron_potential(z2)

    F1 = dum.m1*Ra.dot(U1_grad)
    F2 = dum.m2*Ra.dot(U2_grad)

    M1 = dum.m1 * attitude.hat_map(rho1).dot(R.T.dot(Ra).dot(U1_grad))
    M2 = dum.m2 * attitude.hat_map(rho2).dot(R.T.dot(Ra).dot(U2_grad))
    
    # generate image at this current state only at a specifc time
    # blender.driver(pos, R, ast.omega * t, [5, 0, 1], 'test' + str.zfill(str(t), 4))
    # use the imagery to figure out motion and pass to the controller instead
    # of the true state


    # compute the control input
    u_m = controller.attitude_traverse_then_land_controller(t, state, M1+M2, dum, ast)
    u_f = controller.translation_traverse_then_land_controller(t, state, F1+F2, dum, ast)

    pos_dot = vel
    vel_dot = 1/(dum.m1+dum.m2) *(F1 + F2 + u_f)
    R_dot = R.dot(attitude.hat_map(ang_vel)).reshape(9)
    ang_vel_dot = np.linalg.inv(J).dot(-np.cross(ang_vel,J.dot(ang_vel)) + M1 + M2 + u_m)

    statedot = np.hstack((pos_dot, vel_dot, R_dot, ang_vel_dot))

    return statedot

def blender_traverse_then_land_sim():
    # simulation parameters
    output_path = './visualization/blender'
    asteroid_name = 'itokawa_low'
    # create a HDF5 dataset
    hdf5_path = './data/itokawa_landing/{}_controlled_vertical_landing.hdf5'.format(
        datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"))
    dataset_name = 'landing'
    render = 'BLENDER'
    image_modulus = 400
    RelTol = 1e-6
    AbsTol = 1e-6
    ast_name = 'itokawa'
    num_faces = 64
    t0 = 0
    dt = 1
    tf = 7200
    num_steps = 7200

    periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
    periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # instantiate the blender scene once
    camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene = blender.blender_init(render_engine=render, asteroid_name=asteroid_name)

    # get some of the camera parameters
    K = blender_camera.get_calibration_matrix_K_from_blender(camera)

    # set initial state for inertial EOMs
    # initial_pos = np.array([2.550, 0, 0]) # km for center of mass in body frame
    initial_pos = np.array([0, -2.550, 0])
    initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
    initial_R = attitude.rot3(np.pi/2).reshape(9) # transforms from dumbbell body frame to the inertial frame
    initial_w = np.array([0.01, 0.01, 0.01])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # instantiate ode object
    # system = integrate.ode(eoms_controlled_blender)
    system = integrate.ode(eoms_controlled_blender_traverse_then_land)
    system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1000)
    system.set_initial_value(initial_state, t0)
    system.set_f_params(dum, ast)

    i_state = np.zeros((num_steps+1, 18))
    time = np.zeros(num_steps+1)
    i_state[0, :] = initial_state

    with h5py.File(hdf5_path) as image_data:
        # create a dataset

        images = image_data.create_dataset(dataset_name, (244, 537, 3, num_steps/image_modulus), dtype='uint8')
        RT_blender = image_data.create_dataset('RT', (num_steps/image_modulus, 12))
        R_i2bcam = image_data.create_dataset('R_i2bcam', (num_steps/image_modulus, 9))
        
        ii = 1
        while system.successful() and system.t < tf:
            # integrate the system and save state to an array
            
            time[ii] = (system.t + dt)
            i_state[ii, :] = (system.integrate(system.t + dt))
            # generate the view of the asteroid at this state
            if int(time[ii]) % image_modulus== 0:
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

def blender_vertical_landing_sim():
    # simulation parameters
    output_path = './visualization/blender'
    asteroid_name = 'itokawa_low'
    # create a HDF5 dataset
    hdf5_path = './data/itokawa_landing/{}_controlled_vertical_landing.hdf5'.format(
        datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"))
    dataset_name = 'landing'
    render = 'BLENDER'
    image_modulus = 200
    RelTol = 1e-6
    AbsTol = 1e-6
    ast_name = 'itokawa'
    num_faces = 64
    t0 = 0
    dt = 1
    tf = 3600
    num_steps = 3600

    periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
    periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # instantiate the blender scene once
    camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene = blender.blender_init(render_engine=render, asteroid_name=asteroid_name)

    # get some of the camera parameters
    K = blender_camera.get_calibration_matrix_K_from_blender(camera)

    # set initial state for inertial EOMs
    initial_pos = np.array([2.550, 0, 0]) # km for center of mass in body frame
    initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
    initial_R = attitude.rot3(np.pi).reshape(9) # transforms from dumbbell body frame to the inertial frame
    initial_w = np.array([0.01, 0.01, 0.01])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # instantiate ode object
    system = integrate.ode(eoms_controlled_blender)
    system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1000)
    system.set_initial_value(initial_state, t0)
    system.set_f_params(dum, ast)

    i_state = np.zeros((num_steps+1, 18))
    time = np.zeros(num_steps+1)
    i_state[0, :] = initial_state

    with h5py.File(hdf5_path) as image_data:
        # create a dataset

        images = image_data.create_dataset(dataset_name, (244, 537, 3, num_steps/image_modulus), dtype='uint8')
        RT_blender = image_data.create_dataset('RT', (num_steps/image_modulus, 12))
        R_i2bcam = image_data.create_dataset('R_i2bcam', (num_steps/image_modulus, 9))
        
        ii = 1
        while system.successful() and system.t < tf:
            # integrate the system and save state to an array
            
            time[ii] = (system.t + dt)
            i_state[ii, :] = (system.integrate(system.t + dt))
            # generate the view of the asteroid at this state
            if int(time[ii]) % image_modulus == 0:
                img, RT, R = blender.gen_image(i_state[ii,0:3], i_state[ii,6:15].reshape((3, 3)),
                                ast.omega * time[ii],
                                camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene,
                                [5, 0, 1], 'test')

                images[:, :, :, ii // image_modulus - 1] = img
                RT_blender[ii // image_modulus - 1, :] = RT.reshape(12)
                R_i2bcam[ii // image_modulus - 1, :] = R.reshape(9)


            # do some image processing and visual odometry
            ii += 1

        image_data.create_dataset('K', data=K)
        image_data.create_dataset('i_state', data=i_state)
        image_data.create_dataset('time', data=time)

def blender_inertial_circumnavigate(gen_images=False):
    """Move around the asteroid in the inertial frame, but assume no rotation of the asteroid
    """
    # simulation parameters
    output_path = './visualization/blender'
    asteroid_name = 'itokawa_high'
    # create a HDF5 dataset
    hdf5_path = './data/asteroid_circumnavigate/{}_inertial_no_ast_rotation.hdf5'.format(
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
    tf = 3600 * 4
    num_steps = 3600 * 4
    loops = 4

    periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
    periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # instantiate the blender scene once
    camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene = blender.blender_init(render_engine=render, asteroid_name=asteroid_name)

    # get some of the camera parameters
    K = blender_camera.get_calibration_matrix_K_from_blender(camera)

    # set initial state for inertial EOMs
    initial_pos = np.array([3, 0, 0]) # km for center of mass in body frame
    initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
    initial_R = attitude.rot3(np.pi).reshape(9) # transforms from dumbbell body frame to the inertial frame
    initial_w = np.array([0.01, 0.01, 0.01])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # instantiate ode object
    system = integrate.ode(eoms.eoms_controlled_inertial_circumnavigate)
    system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1000)
    system.set_initial_value(initial_state, t0)
    system.set_f_params(dum, ast, tf, loops)

    i_state = np.zeros((num_steps+1, 18))
    time = np.zeros(num_steps+1)
    i_state[0, :] = initial_state

    with h5py.File(hdf5_path) as image_data:
        # create a dataset
        if gen_images:
            images = image_data.create_dataset(dataset_name, (244, 537, 3, num_steps/image_modulus), dtype='uint8')
            RT_blender = image_data.create_dataset('RT', (num_steps/image_modulus, 12))
            R_i2bcam = image_data.create_dataset('R_i2bcam', (num_steps/image_modulus, 9))
        
        ii = 1
        while system.successful() and system.t < tf:
            # integrate the system and save state to an array
            
            time[ii] = (system.t + dt)
            i_state[ii, :] = (system.integrate(system.t + dt))
            # generate the view of the asteroid at this state
            if int(time[ii]) % image_modulus == 0 and gen_images:
                # img, RT, R = blender.gen_image(i_state[ii,0:3], i_state[ii,6:15].reshape((3, 3)),
                #                 ast.omega * time[ii],
                #                 camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene,
                #                 [5, 0, 1], 'test')
                img, RT, R = blender.gen_image_fixed_ast(i_state[ii,0:3], 
                                                         i_state[ii,6:15].reshape((3,3)),
                                                         camera_obj, camera,
                                                         lamp_obj, lamp,
                                                         itokawa_obj, scene,
                                                         [5, 0, 1], 'test')

                images[:, :, :, ii // image_modulus - 1] = img
                RT_blender[ii // image_modulus - 1, :] = RT.reshape(12)
                R_i2bcam[ii // image_modulus - 1, :] = R.reshape(9)


            # do some image processing and visual odometry
            ii += 1

        image_data.create_dataset('K', data=K)
        image_data.create_dataset('i_state', data=i_state)
        image_data.create_dataset('time', data=time)

def blender_inertial_lissajous(gen_images=False):
    """Move around the asteroid in the inertial frame, but assume no rotation of the asteroid
    """
    # simulation parameters
    output_path = './visualization/blender'
    asteroid_name = 'itokawa_high'
    # create a HDF5 dataset
    hdf5_path = './data/asteroid_circumnavigate/{}_inertial_no_ast_rotation_lissajous.hdf5'.format(
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
    tf = 3600 * 2
    num_steps = 3600 * 2
    loops = 2

    periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
    periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # instantiate the blender scene once
    camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene = blender.blender_init(render_engine=render, asteroid_name=asteroid_name)

    # get some of the camera parameters
    K = blender_camera.get_calibration_matrix_K_from_blender(camera)

    # set initial state for inertial EOMs
    initial_pos = np.array([3, 3, 0]) # km for center of mass in body frame
    initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
    initial_R = attitude.rot3(np.pi).reshape(9) # transforms from dumbbell body frame to the inertial frame
    initial_w = np.array([0.01, 0.01, 0.01])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # instantiate ode object
    system = integrate.ode(eoms.eoms_controlled_inertial_lissajous)
    system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1000)
    system.set_initial_value(initial_state, t0)
    system.set_f_params(dum, ast, tf, loops)

    i_state = np.zeros((num_steps+1, 18))
    time = np.zeros(num_steps+1)
    i_state[0, :] = initial_state

    with h5py.File(hdf5_path) as image_data:
        # create a dataset
        if gen_images:
            images = image_data.create_dataset(dataset_name, (244, 537, 3, num_steps/image_modulus), dtype='uint8')
            RT_blender = image_data.create_dataset('RT', (num_steps/image_modulus, 12))
            R_i2bcam = image_data.create_dataset('R_i2bcam', (num_steps/image_modulus, 9))
        
        ii = 1
        while system.successful() and system.t < tf:
            # integrate the system and save state to an array
            
            time[ii] = (system.t + dt)
            i_state[ii, :] = (system.integrate(system.t + dt))
            # generate the view of the asteroid at this state
            if int(time[ii]) % image_modulus == 0 and gen_images:
                # img, RT, R = blender.gen_image(i_state[ii,0:3], i_state[ii,6:15].reshape((3, 3)),
                #                 ast.omega * time[ii],
                #                 camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene,
                #                 [5, 0, 1], 'test')
                img, RT, R = blender.gen_image_fixed_ast(i_state[ii,0:3], 
                                                         i_state[ii,6:15].reshape((3,3)),
                                                         camera_obj, camera,
                                                         lamp_obj, lamp,
                                                         itokawa_obj, scene,
                                                         [5, 0, 1], 'test')

                images[:, :, :, ii // image_modulus - 1] = img
                RT_blender[ii // image_modulus - 1, :] = RT.reshape(12)
                R_i2bcam[ii // image_modulus - 1, :] = R.reshape(9)


            # do some image processing and visual odometry
            ii += 1

        image_data.create_dataset('K', data=K)
        image_data.create_dataset('i_state', data=i_state)
        image_data.create_dataset('time', data=time)

def blender_inertial_quarter_equatorial(gen_images=False):
    """Move around the asteroid in the inertial frame, but assume no rotation of the asteroid

    Moves in the xy positive quadrant in the equatorial plane
    """
    # simulation parameters
    output_path = './visualization/blender'
    asteroid_name = 'itokawa_high'
    # create a HDF5 dataset
    hdf5_path = './data/asteroid_circumnavigate/{}_inertial_no_ast_rotation_quarter_xy.hdf5'.format(
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
    tf = 3600 * 4
    num_steps = 3600 * 4
    loops = 4

    periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
    periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # instantiate the blender scene once
    camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene = blender.blender_init(render_engine=render, asteroid_name=asteroid_name)

    # get some of the camera parameters
    K = blender_camera.get_calibration_matrix_K_from_blender(camera)

    # set initial state for inertial EOMs
    initial_pos = np.array([3, 0, 0]) # km for center of mass in body frame
    initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
    initial_R = attitude.rot3(np.pi).reshape(9) # transforms from dumbbell body frame to the inertial frame
    initial_w = np.array([0.01, 0.01, 0.01])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # instantiate ode object
    system = integrate.ode(eoms.eoms_controlled_inertial_quarter_equatorial)
    system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1000)
    system.set_initial_value(initial_state, t0)
    system.set_f_params(dum, ast, tf, loops)

    i_state = np.zeros((num_steps+1, 18))
    time = np.zeros(num_steps+1)
    i_state[0, :] = initial_state

    with h5py.File(hdf5_path) as image_data:
        # create a dataset
        if gen_images:
            images = image_data.create_dataset(dataset_name, (244, 537, 3, num_steps/image_modulus), dtype='uint8')
            RT_blender = image_data.create_dataset('RT', (num_steps/image_modulus, 12))
            R_i2bcam = image_data.create_dataset('R_i2bcam', (num_steps/image_modulus, 9))
        
        ii = 1
        while system.successful() and system.t < tf:
            # integrate the system and save state to an array
            
            time[ii] = (system.t + dt)
            i_state[ii, :] = (system.integrate(system.t + dt))
            # generate the view of the asteroid at this state
            if int(time[ii]) % image_modulus == 0 and gen_images:
                # img, RT, R = blender.gen_image(i_state[ii,0:3], i_state[ii,6:15].reshape((3, 3)),
                #                 ast.omega * time[ii],
                #                 camera_obj, camera, lamp_obj, lamp, itokawa_obj, scene,
                #                 [5, 0, 1], 'test')
                img, RT, R = blender.gen_image_fixed_ast(i_state[ii,0:3], 
                                                         i_state[ii,6:15].reshape((3,3)),
                                                         camera_obj, camera,
                                                         lamp_obj, lamp,
                                                         itokawa_obj, scene,
                                                         [5, 0, 1], 'test')

                images[:, :, :, ii // image_modulus - 1] = img
                RT_blender[ii // image_modulus - 1, :] = RT.reshape(12)
                R_i2bcam[ii // image_modulus - 1, :] = R.reshape(9)


            # do some image processing and visual odometry
            ii += 1

        image_data.create_dataset('K', data=K)
        image_data.create_dataset('i_state', data=i_state)
        image_data.create_dataset('time', data=time)
