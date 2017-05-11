"""Simulation of controlled dumbbell around an asteroid
"""
from scipy import integrate
import numpy as np

import visualization.plotting as plotting
import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
from kinematics import attitude

import inertial_driver as idriver
import relative_driver as rdriver

RelTol = 1e-9
AbsTol = 1e-9
ast_name = 'castalia'
num_faces = 64
tf = 1e2
num_steps = 1e2
time = np.linspace(0,tf,num_steps)

periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

ast = asteroid.Asteroid(ast_name,num_faces)
dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

# set initial state for inertial EOMs
initial_pos = periodic_pos # km for center of mass in body frame
initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
initial_R = attitude.rot2(np.pi/2).reshape(9) # transforms from dumbbell body frame to the inertial frame
initial_w = np.array([0.01, 0.01, 0.01])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

i_state = integrate.odeint(dum.eoms_inertial_control, initial_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)
