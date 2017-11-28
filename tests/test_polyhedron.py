"""Test polyhedron potential functions
"""
import numpy as np
from dynamics import asteroid
from point_cloud import polyhedron
import pdb

# TODO: Put all of the OBJ based testing into a single class. This avoids having to instantiate many large asteroid objects
class TestItokawaMat():
    # define asteroid
    ast = asteroid.Asteroid('itokawa', 32, 'mat')
    pos = np.random.uniform(1, 2, size=(3,))
    V = ast.V
    num_v = V.shape[0]

    r_v = V - np.tile(pos, (num_v, 1))
    Fa, Fb, Fc = ast.asteroid_grav['Fa'], ast.asteroid_grav['Fb'], ast.asteroid_grav['Fc']
    F_face = ast.asteroid_grav['F_face']
    w_face = polyhedron.laplacian_factor(r_v, Fa, Fb, Fc)
    # call both functions
    U_face_loop, U_grad_face_loop, U_grad_mat_face_loop = polyhedron.face_contribution_loop(r_v, Fa, F_face, w_face)
    U_face, U_grad_face, U_grad_mat_face= polyhedron.face_contribution(r_v, Fa, F_face, w_face)

    # make sure things match
    def test_potential(self):
        np.testing.assert_allclose(self.U_face, self.U_face_loop)

    def test_attraction(self):
        np.testing.assert_allclose(self.U_grad_face, self.U_grad_face_loop)

    def test_laplacian(self):
        np.testing.assert_allclose(self.U_grad_mat_face, self.U_grad_mat_face_loop)

class TestItokawaLowOBJ():
    # define asteroid
    ast = asteroid.Asteroid('itokawa', 0, 'obj')
    pos = np.random.uniform(1, 2, size=(3,))
    V = ast.V
    num_v = V.shape[0]

    r_v = V - np.tile(pos, (num_v, 1))

    # extract things from object
    Fa, Fb, Fc = ast.asteroid_grav['Fa'], ast.asteroid_grav['Fb'], ast.asteroid_grav['Fc']
    F_face = ast.asteroid_grav['F_face']
    e1, e2, e3 = ast.asteroid_grav['e1'], ast.asteroid_grav['e2'], ast.asteroid_grav['e3']
    e1_vertex_map, e2_vertex_map, e3_vertex_map = ast.asteroid_grav['e1_vertex_map'], ast.asteroid_grav['e2_vertex_map'], ast.asteroid_grav['e3_vertex_map']
    e1_face_map, e2_face_map, e3_face_map = ast.asteroid_grav['e1_face_map'], ast.asteroid_grav['e2_face_map'], ast.asteroid_grav['e3_face_map']
    E1_edge, E2_edge, E3_edge = ast.asteroid_grav['E1_edge'], ast.asteroid_grav['E2_edge'], ast.asteroid_grav['E3_edge']
    e_vertex_map, unique_index = ast.asteroid_grav['e_vertex_map'], ast.asteroid_grav['unique_index']

    w_face = polyhedron.laplacian_factor(r_v, Fa, Fb, Fc)

    # compute things to compare methods
    L1_edge, L2_edge, L3_edge = polyhedron.edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map)
    L1_edge_map, _, _ = polyhedron.map_edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map)

    U_face_loop, U_grad_face_loop, U_grad_mat_face_loop = polyhedron.face_contribution_loop(r_v, Fa, F_face, w_face)
    U_face, U_grad_face, U_grad_mat_face= polyhedron.face_contribution(r_v, Fa, F_face, w_face)
    
    U_edge_loop, U_grad_edge_loop, U_grad_mat_edge_loop = polyhedron.edge_contribution_loop(r_v, e1_face_map, e2_face_map, e3_face_map, e1_vertex_map, e2_vertex_map, e3_vertex_map, E1_edge, E2_edge, E3_edge, L1_edge, L2_edge, L3_edge) 
    U_edge, U_grad_edge, U_grad_mat_edge = polyhedron.edge_contribution(pos, e_vertex_map, unique_index, V, E1_edge, E2_edge, E3_edge, L1_edge, L2_edge, L3_edge)
    

    # make sure things match
    def test_potential(self):
        np.testing.assert_allclose(self.U_face, self.U_face_loop)

    def test_attraction(self):
        np.testing.assert_allclose(self.U_grad_face, self.U_grad_face_loop)

    def test_laplacian(self):
        np.testing.assert_allclose(self.U_grad_mat_face, self.U_grad_mat_face_loop)

    def test_L1_edge(self):
        np.testing.assert_allclose(self.L1_edge_map, self.L1_edge)




