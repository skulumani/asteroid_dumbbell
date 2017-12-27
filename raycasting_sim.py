"""Simulation of a spacecraft with a LIDAR taking measurements 
around an asteroid
"""
from __future__ import absolute_import, division, print_function, unicode_literals
from dynamics import asteroid, dumbbell, eoms
from kinematics import attitude
from visualization import plotting, graphics

import numpy as np
from scipy import integrate

# create the sensor and raycaster

# create an asteroid and dumbbell

# simulate dumbbell moving aroudn asteroid
ast = asteroid.Asteroid('castalia', 4092, 'mat')
dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

time = np.linspace(0, int(1e4), int(1e4))
initial_pos = np.array([1.5, 0, 0])
initial_vel = np.array([0, 0, 0])
initial_R = np.eye(3,3).reshape(-1)
initial_w = np.array([0, 0, 0])
initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

# try both a controlled and uncontrolled simulation
# t, istate, astate, bstate = eoms.inertial_eoms_driver(initial_state, time, ast, dum)

state = eoms.sim_controlled_driver(time, initial_state, ast, dum)
# plot the simulation
# plotting.animate_inertial_trajectory(t, istate, ast, dum)
# plotting.plot_inertial(t, istate, ast, dum, fwidth=1)

# TODO: animation in mayavi
mfig = graphics.mayavi_figure()

graphics.draw_polyhedron_mayavi(ast.V, ast.F, mfig)
graphics.mayavi_plot_trajectory(mfig, state[:, 0:3])
