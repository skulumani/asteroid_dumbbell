# Script to compare equations of motion
import eom_comparison.utilities as eom

import visualization.plotting as plotting
import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
from kinematics import attitude

import inertial_driver as idriver
import relative_driver as rdriver
from scipy import integrate
import numpy as np
import pdb

RelTol = 1e-9
AbsTol = 1e-9
ast_name = 'castalia'
num_faces = 64
tf = 1e3
num_steps = 1e3
time = np.linspace(0,tf,num_steps)

periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

ast = asteroid.Asteroid(ast_name,num_faces)
dum = dumbbell.Dumbbell(m1=10000, m2=10000, l=0.003)

print("Running inertial EOMS")
# set initial state for inertial EOMs
initial_pos = periodic_pos # km for center of mass in body frame
initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
initial_R = attitude.rot2(np.pi/2).reshape(9) # transforms from dumbbell body frame to the inertial frame
initial_w = np.array([0.00, 0.01, 0.00])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

i_state = integrate.odeint(dum.eoms_inertial, initial_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)

print("Running hamiltonian asteroid EOMs")
initial_lin_mom = (dum.m1 + dum.m2) * (periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos))
initial_ang_mom = initial_R.reshape((3,3)).dot(dum.J).dot(initial_w)
initial_ham_state = np.hstack((initial_pos, initial_lin_mom, initial_R, initial_ang_mom))

rh_state = integrate.odeint(dum.eoms_hamilton_relative, initial_ham_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)

# do the inverse legendre transformation
rel_lin_mom = rh_state[:, 3:6]
rel_ang_mom = rh_state[:, 15:18]

rh_vel = rel_lin_mom / (dum.m1 + dum.m2)
rh_ang_vel = np.zeros_like(rel_ang_mom)
# convert the angular momentum into the equivalent angular velocity
for ii in range(rh_state.shape[0]):
    R = rh_state[ii, 6:15].reshape((3,3))    

    Jr = R.dot(dum.J).dot(R.T)
    rh_ang_vel[ii, :] = np.linalg.inv(Jr).dot(rel_ang_mom[ii, :])

rh_state_conv = np.hstack((rh_state[:, 0:3], rh_vel, rh_state[:, 6:15], rh_ang_vel))

# also compute and compare the energy behavior
print("Computing inertial energy")
i_KE, i_PE = dum.inertial_energy(time, i_state, ast)

print("Computing asteroid energy")
r_KE, r_PE = dum.relative_energy(time, rh_state_conv, ast)
 
plotting.plot_energy(time, i_KE, i_PE)

print("Plot comparison in the inertial frame")
plotting.plot_inertial_comparison(time,time, rh_state_conv, i_state, ast, dum)
