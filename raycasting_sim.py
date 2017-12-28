"""Simulation of a spacecraft with a LIDAR taking measurements 
around an asteroid
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import pdb

import numpy as np
from scipy import integrate

from dynamics import asteroid, dumbbell, eoms, controller
from kinematics import attitude
from visualization import plotting, graphics, animation
from point_cloud import wavefront, raycaster

# simulate dumbbell moving aroudn asteroid
ast = asteroid.Asteroid('castalia', 4092, 'mat')
dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
des_att_func = controller.body_fixed_pointing_attitude
des_tran_func = controller.inertial_fixed_state
AbsTol = 1e-9
RelTol = 1e-9

num_steps = int(1e4)
time = np.linspace(0, num_steps, num_steps)
t0, tf = time[0], time[-1]
dt = time[1] - time[0]

initial_pos = np.array([1.5, 0, 0])
initial_vel = np.array([0, 0, 0])
initial_R = attitude.rot3(np.pi/2).reshape(-1)
initial_w = np.array([0, 0, 0])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

# initialize the raycaster and lidar
polydata = wavefront.meshtopolydata(ast.V, ast.F)
caster = raycaster.RayCaster(polydata)
sensor = raycaster.Lidar(dist=5)
# try both a controlled and uncontrolled simulation
# t, istate, astate, bstate = eoms.inertial_eoms_driver(initial_state, time, ast, dum)

system = integrate.ode(eoms.eoms_controlled_inertial)
system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=num_steps)
system.set_initial_value(initial_state, t0)
system.set_f_params(ast, dum, des_att_func, des_tran_func)

# TODO Create a point cloud dictionary to hold intersections, sc state, time, asateroid state
state = np.zeros((num_steps+1, 18))
t = np.zeros(num_steps+1)
int_array = []
state[0, :] = initial_state

ii = 1
# TODO Need to ensure that the pointing vectors are consistent (body frame, controller, sensor)
while system.successful() and system.t < tf:
    # integrate the system and save state to an array
    t[ii] = (system.t + dt)
    state[ii, :] = system.integrate(system.t + dt)
    # create the sensor and raycaster
    targets = state[ii, 0:3] + np.linalg.norm(state[ii, 0:3]) * sensor.rotate_fov(state[ii, 6:15].reshape((3,3)))

    # TODO Need to update the caster with the rotated asteroid
    intersections = caster.castarray(state[ii, 0:3], targets)
    int_array.append(intersections)
    # create an asteroid and dumbbell
    ii+= 1

# plot the simulation
# plotting.animate_inertial_trajectory(t, istate, ast, dum)
# plotting.plot_controlled_inertial(t, istate, ast, dum, fwidth=1)

mfig = graphics.mayavi_figure() 
mesh, ast_axes = graphics.draw_polyhedron_mayavi(ast.V, ast.F, mfig)

com, dum_axes = graphics.draw_dumbbell_mayavi(state[0, :], dum, mfig)

# TODO draw all the sensor raycasting lines
animation.inertial_asteroid_trajectory(time, state, ast, dum, (mesh, ast_axes,
                                                               com, dum_axes))
