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

initial_w = np.array([0.01, 0.0, 0.0])
print("Running inertial EOMS")
i_time, i_state = idriver.inertial_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)

print("Running asteroid EOMs")
r_time, r_state = rdriver.relative_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)

ast = asteroid.Asteroid(ast_name,num_faces)
dum = dumbbell.Dumbbell()

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
plotting.plot_inertial_comparison(r_time, i_time, r_state, i_state, ast, dum) 

