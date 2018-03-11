from dynamics import asteroid
import numpy as np
from lib import polyhedron_potential, binding

# create the asteroid
ast = asteroid.Asteroid('castalia', 0, 'obj')

# extract out some things
V = ast.asteroid_grav['V']
F = ast.asteroid_grav['F']
Fa = ast.asteroid_grav['Fa']
Fb = ast.asteroid_grav['Fb']
Fc = ast.asteroid_grav['Fc']
F_face = ast.asteroid_grav['F_face']
num_v = V.shape[0]

state = np.array([2, 0, 0])
r_v =  V - np.tile(state, (num_v, 1))

w_face = polyhedron_potential.laplacian_factor(binding.numpy_to_eigen(r_v, dtype=np.float64),
                                               binding.numpy_to_eigen(Fa.reshape((-1, 1)), dtype=np.int32),
                                               binding.numpy_to_eigen(Fb.reshape((-1, 1)), dtype=np.int32),
                                               binding.numpy_to_eigen(Fc.reshape((-1, 1)), dtype=np.int32))

# now try the compiled library
# polyhedron_potential.face_contribution_loop(state, V, F, F_face, w_face)
