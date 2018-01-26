import pdb

import numpy as np
from kinematics import sphere

from dynamics import asteroid
from point_cloud import wavefront



class TestDistanceToEdgesCubeOutsideFixedSingle():

    pt_out=np.array([1, 0.5, 0.5])
    ast=asteroid.Asteroid('castalia', 256, 'mat')
    v, f=wavefront.read_obj('./integration/cube.obj')
    ast=ast.loadmesh(v, f, 'cube')

    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_edges(pt_out, v, f,
                                                normal_face, 
                                                edge_vertex_map,
                                                edge_face_map,
                                                vf_map)
    D_exp = 0.5
    P_exp = np.array([0.5, 0.5, 0.5])
    V_exp = [4, 7]
    E_exp = [7, 4]
    F_exp = [6, 7]

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

class TestDistanceToEdgesCubeOutsideFixedMultiple():
    """This point is equally close to vertex so hopefully multiple edges
    have equal distance
    """
    pt_out = np.array([1, 0, 0])
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')
    
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']


    D, P, V, E, F = wavefront.distance_to_edges(pt_out,v,f,
                                                normal_face, 
                                                edge_vertex_map,
                                                edge_face_map,
                                                vf_map)
    D_exp = np.array([0.5, 0.5]) 
    P_exp = np.array([[0.5, -0, -0],
                      [0.5, 0, 0]])
    V_exp = np.array([4, 7])
    E_exp = np.array([[7, 4],
                      [4, 7]])
    F_exp = np.array([[6, 7],
                      [6, 7]])

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
