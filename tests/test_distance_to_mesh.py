"""Test out the combined distance to mesh function
"""
import pdb

import numpy as np
from kinematics import sphere

from dynamics import asteroid
from point_cloud import wavefront


class TestDistanceToMeshCubeEdge():

    v, f = wavefront.read_obj('./integration/cube.obj')
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    pt = np.array([1, 0.5, 0.1])
    D, P, V, E, F, primitive = wavefront.distance_to_mesh(
        pt, v, f, mesh_parameters)

    D_exp = 0.5
    P_exp = np.array([0.5, 0.5, 0.1])
    V_exp = np.array([6, 7])
    E_exp = np.array([6, 7])
    F_exp = np.array([4, 6])
    primitive_exp = 'edge'

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)

    def test_point(self):
        np.testing.assert_array_almost_equal(self.P, self.P_exp)

    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

    def test_edge(self):
        np.testing.assert_allclose(self.E, self.E_exp)

    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)
    def test_primitive(self):
        np.testing.assert_allclose(self.primitive, self.primitive_exp)


class TestDistanceToMeshCubeVertex():

    v, f = wavefront.read_obj('./integration/cube.obj')
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    pt = np.array([1, 0.5, 0.5])
    D, P, V, E, F, primitive = wavefront.distance_to_mesh(
        pt, v, f, mesh_parameters)

    D_exp = 0.5
    P_exp = np.array([0.5, 0.5, 0.5])
    V_exp = np.array(7)
    E_exp = np.array([[7, 2],
                      [7, 4],
                      [7, 1],
                      [6, 7],
                      [7, 3],
                      [7, 6],
                      [5, 7],
                      [7, 5],
                      [3, 7],
                      [2, 7],
                      [4, 7],
                      [1, 7]])
    F_exp = np.array([4, 5, 6, 7, 10, 11])
    primitive_exp = 'vertex'

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)

    def test_point(self):
        np.testing.assert_array_almost_equal(self.P, self.P_exp)

    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

    def test_edge(self):
        np.testing.assert_allclose(self.E, self.E_exp)

    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)

    def test_primitive(self):
        np.testing.assert_allclose(self.primitive, self.primitive_exp)

class TestDistanceToMeshCubeFace():

    v, f = wavefront.read_obj('./integration/cube.obj')
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    pt = np.array([1, 0.5, 0.5])
    D, P, V, E, F, primitive = wavefront.distance_to_mesh(
        pt, v, f, mesh_parameters)

    D_exp = 0.5
    P_exp = np.array([0.5, 0.2, 0])
    V_exp = np.array([4, 6, 7])
    E_exp = np.array([[6, 4],
                      [7, 6],
                      [4, 7]])
    F_exp = np.array(6)
    primitive_exp = 'face'

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)

    def test_point(self):
        np.testing.assert_array_almost_equal(self.P, self.P_exp)

    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

    def test_edge(self):
        np.testing.assert_allclose(self.E, self.E_exp)

    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)

    def test_primitive(self):
        np.testing.assert_allclose(self.primitive, self.primitive_exp)
