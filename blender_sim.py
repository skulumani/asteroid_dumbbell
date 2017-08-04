"""Simulation of controlled dumbbell around Itokawa with 
simulated imagery using Blender

4 August 2017 - Shankar Kulumani
"""
from __future__ import absolute_import, division, print_function, unicode_literals
from scipy import integrate
import numpy as np
import pdb

import visualization.plotting as plotting
import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
from kinematics import attitude

import inertial_driver as idriver
import relative_driver as rdriver

RelTol = 1e-6
AbsTol = 1e-6
ast_name = 'itokawa'
num_faces = 64
tf = 1e3
num_steps = 1e3
time = np.linspace(0,int(tf),int(num_steps))

periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

ast = asteroid.Asteroid(ast_name,num_faces)
dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

t0 = 0
tf = 1e2
dt = 1
num_steps = 100

# set initial state for inertial EOMs
initial_pos = periodic_pos # km for center of mass in body frame
initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
initial_R = attitude.rot2(np.pi/2).reshape(9) # transforms from dumbbell body frame to the inertial frame
initial_w = np.array([0.01, 0.01, 0.01])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

# instantiate ode object
system = integrate.ode(dum.eoms_inertial_control_ode)
system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1000)
system.set_initial_value(initial_state, t0)
system.set_f_params(ast)

i_state = np.zeros((num_steps+1, 18))
time = np.zeros(num_steps+1)

ii = 1
while system.successful() and system.t < tf:
    # integrate the system and save state to an array
    
    time[ii] = (system.t + dt)
    i_state[ii, :] = (system.integrate(system.t + dt))
    # generate the view of the asteroid at this state

    # do some image processing and visual odometry
    ii += 1
