"""Simulation of a spacecraft with a LIDAR taking measurements 
around an asteroid
"""
from __future__ import absolute_import, division, print_function, unicode_literals
from dynamics import asteroid, dumbbell, eoms
from kinematics import attitude

import numpy as np
from scipy import integrate

# create the sensor and raycaster

# create an asteroid and dumbbell

# simulate dumbbell moving aroudn asteroid
ast = asteroid.Asteroid('castalia', 4092, 'mat')
dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

time = np.linspace(0, 1000, 1000)
initial_pos = np.array([1.5, 0, 0])
initial_vel = np.array([0, 0, 0])
initial_R = np.eye(3,3).reshape(-1)
initial_w = np.array([0, 0, 0])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

t, istate, astate, bstate = eoms.inertial_eoms_driver(initial_state,
                                                      time, ast, dum)

# plot the simulation

# animation in mayavi

