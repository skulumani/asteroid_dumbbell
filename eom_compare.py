"""Compare the equations of motion and verify their validity

This module will simulate a dumbbell around an asteroid using both the inertial
and relative equations of motion. The motion is then compared to ensure that both
are valid.
"""
from scipy import integrate
import numpy as np
import pdb
import eom_comparison.utilities as eom

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
tf = 1e3
num_steps = 1e5
time = np.linspace(0,tf,num_steps)

periodic_pos = np.array([1.495746722510590,0.000001002669660,0.006129720493607])
periodic_vel = np.array([0.000000302161724,-0.000899607989820,-0.000000013286327])

ast = asteroid.Asteroid(ast_name,num_faces)
dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

print("Running inertial EOMS")
# set initial state for inertial EOMs
initial_pos = periodic_pos # km for center of mass in body frame
initial_vel = periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos)
initial_R = attitude.rot2(np.pi/2).reshape(9) # transforms from dumbbell body frame to the inertial frame
initial_w = np.array([0.01, 0.01, 0.01])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

def run_inertial():
    i_state = integrate.odeint(dum.eoms_inertial, initial_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)
    # (i_time, i_state) = idriver.eom_inertial_driver(initial_state, time, ast, dum, AbsTol=1e-9, RelTol=1e-9)
    return i_state

print("Running hamiltonian asteroid EOMs")
initial_lin_mom = (dum.m1 + dum.m2) * (periodic_vel + attitude.hat_map(ast.omega*np.array([0,0,1])).dot(initial_pos))
initial_ang_mom = initial_R.reshape((3,3)).dot(dum.J).dot(initial_w)
initial_ham_state = np.hstack((initial_pos, initial_lin_mom, initial_R, initial_ang_mom))

def run_hamilton():
    rh_state = integrate.odeint(dum.eoms_hamilton_relative, initial_ham_state, time, args=(ast,), atol=AbsTol, rtol=RelTol)
    # (rh_time, rh_state) = rdriver.eoms_hamilton_relative_driver(initial_ham_state, time, ast, dum, AbsTol=1e-9, RelTol=1e-9)
    return rh_state

i_state = run_inertial()
rh_state = run_hamilton()
# also compute and compare the energy behavior
# print("Computing inertial energy")
# i_KE, i_PE = dum.inertial_energy(time, i_state, ast)

# print("Computing asteroid energy")
# r_KE, r_PE = dum.relative_energy(time, rh_state_conv, ast)
 
# plotting.plot_energy(time, i_KE, i_PE)

print("Plot comparison in the inertial frame")
plotting.plot_inertial_comparison(time,time, rh_state, i_state, ast, dum, False, 1)

print("Plot comparison in the asteroid frame")
plotting.plot_asteroid_comparison(time, time, rh_state, i_state, ast, dum, False, 1)
