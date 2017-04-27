# Script to compare equations of motion
import eom_comparison.utilities as eom

import visualization.plotting as plotting
import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell

import inertial_driver as idriver
import relative_driver as rdriver
import numpy as np
import pdb

ast_name = 'castalia'
num_faces = 64
tf = 1e2
num_steps = 1e2

initial_w = np.array([0.0, 0.8, 0])
print("Running inertial EOMS")
i_time, i_state = idriver.inertial_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)

print("Running asteroid EOMs")
r_time, r_state = rdriver.relative_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)

print("Running hamiltonian asteroid EOMs")
rh_time, rh_state = rdriver.relative_hamiltonian_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)

ast = asteroid.Asteroid(ast_name,num_faces)
dum = dumbbell.Dumbbell()

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
# print("Computing inertial energy")
# i_KE, i_PE = dum.inertial_energy(i_time, i_state, ast)
# 
# print("Computing asteroid energy")
# r_KE, r_PE = dum.relative_energy(r_time, r_state, ast)
# 
# plotting.plot_energy(i_time, i_KE, i_PE)
# # also look at the animation of both and the converted form as well

print("Plot comparison in the inertial frame")
# plotting.plot_inertial_comparison(r_time, i_time, r_state, i_state, ast, dum) 
plotting.plot_inertial_comparison(rh_time, i_time, rh_state_conv, i_state, ast, dum)
