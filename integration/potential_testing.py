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

e1 = ast.asteroid_grav['e1']
e2 = ast.asteroid_grav['e2']
e3 = ast.asteroid_grav['e3']
e1_vertex_map = ast.asteroid_grav['e1_vertex_map']
e2_vertex_map = ast.asteroid_grav['e2_vertex_map']
e3_vertex_map = ast.asteroid_grav['e3_vertex_map']

num_v = V.shape[0]

state = np.array([2, 0, 0])
r_v = V - np.tile(state, (num_v, 1))

w_face = np.zeros((1, 1))
flag = polyhedron_potential.laplacian_factor(r_v, Fa, Fb, Fc, w_face)
w_face_python = polyhedron.laplacian_factor(r_v, Fa, Fb, Fc)

# edge factor testing
L1_edge = np.zeros((1, 1))
L2_edge = np.zeros((1,1))
L3_edge = np.zeros((1,1))
flag = polyhedron_potential.edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map, L1_edge, L2_edge, L3_edge)

# now try the compiled library
# polyhedron_potential.face_contribution_loop(state, V, F, F_face, w_face)
