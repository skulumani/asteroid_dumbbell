"""Test polyhedron potential functions
"""
import numpy as np
from dynamics import asteroid
from point_cloud import polyhedron
import pdb
class TestFaceContributionItokawaMat():
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
    U_face, U_grad_face, U_grad_mat_face= polyhedron.face_contribution_loop(r_v, Fa, F_face, w_face)

    # make sure things match
    def test_potential(self):
        np.testing.assert_allclose(self.U_face, self.U_face_loop)

    def test_attraction(self):
        np.testing.assert_allclose(self.U_grad_face, self.U_grad_face_loop)

    def test_laplacian(self):
        np.testing.assert_allclose(self.U_grad_mat_face, self.U_grad_mat_face_loop)

class TestFaceContributionItokawaLowOBJ():
    # define asteroid
    ast = asteroid.Asteroid('itokawa', 0, 'obj')
    pos = np.random.uniform(1, 2, size=(3,))
    V = ast.V
    num_v = V.shape[0]

    r_v = V - np.tile(pos, (num_v, 1))
    Fa, Fb, Fc = ast.asteroid_grav['Fa'], ast.asteroid_grav['Fb'], ast.asteroid_grav['Fc']
    F_face = ast.asteroid_grav['F_face']
    w_face = polyhedron.laplacian_factor(r_v, Fa, Fb, Fc)
    # call both functions
    U_face_loop, U_grad_face_loop, U_grad_mat_face_loop = polyhedron.face_contribution_loop(r_v, Fa, F_face, w_face)
    U_face, U_grad_face, U_grad_mat_face= polyhedron.face_contribution_loop(r_v, Fa, F_face, w_face)

    # make sure things match
    def test_potential(self):
        np.testing.assert_allclose(self.U_face, self.U_face_loop)

    def test_attraction(self):
        np.testing.assert_allclose(self.U_grad_face, self.U_grad_face_loop)

    def test_laplacian(self):
        np.testing.assert_allclose(self.U_grad_mat_face, self.U_grad_mat_face_loop)

class TestEdgeFactorItokawaOBJ():
    ast = asteroid.Asteroid('itokawa', 0, 'obj')
    pos = np.random.uniform(1, 2, size=(3,))
    V = ast.V
    num_v = V.shape[0]

    r_v = V - np.tile(pos, (num_v, 1))
    e1, e2, e3 = ast.asteroid_grav['e1'], ast.asteroid_grav['e2'], ast.asteroid_grav['e3']
    e1_vertex_map, e2_vertex_map, e3_vertex_map = ast.asteroid_grav['e1_vertex_map'], ast.asteroid_grav['e2_vertex_map'], ast.asteroid_grav['e3_vertex_map']
    
    L1_edge, L2_edge, L3_edge = polyhedron.edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map)
    L1_edge_map, _, _ = polyhedron.map_edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map)

    def test_L1_edge(self):
        np.testing.assert_allclose(self.L1_edge_map, self.L1_edge)

