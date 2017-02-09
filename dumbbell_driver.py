# Run the simulation

import dynamics.asteroid
import numpy as np

fontsize = 18;
fontname = 'Times'

periodic_tspan = np.array([0,30000])
RelTol = 1e-9
AbsTol = 1e-9
tol = 1e-6
diffcorr_plot = 0
jacobi_step = 1e-9

ast = dynamics.asteroid.Asteroid('castalia',32)

state = np.array([1.5,0,0])

ast.polyhedron_potential(state)