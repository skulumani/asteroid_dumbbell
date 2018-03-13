from dynamics import asteroid
import numpy as np
from lib import polyhedron_potential, binding
from point_cloud import polyhedron

# create the asteroid
ast = asteroid.Asteroid('cube', 0, 'obj')

# extract out some things
V = ast.asteroid_grav['V']
F = ast.asteroid_grav['F']
Fa = ast.asteroid_grav['Fa']
Fb = ast.asteroid_grav['Fb']
Fc = ast.asteroid_grav['Fc']
F_face = ast.asteroid_grav['F_face']
num_v = V.shape[0]

state = np.array([2, 0, 0])
r_v = V - np.tile(state, (num_v, 1))

w_face = binding.numpy_to_eigen(np.zeros((Fa.shape[0], 1)), dtype=np.float64)
flag = polyhedron_potential.laplacian_factor(r_v, Fa, Fb, Fc, w_face)

w_face_python = polyhedron.laplacian_factor(r_v, Fa, Fb, Fc)

# now try the compiled library
# polyhedron_potential.face_contribution_loop(state, V, F, F_face, w_face)
