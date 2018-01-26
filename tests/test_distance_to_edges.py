import pdb

import numpy as np
from kinematics import sphere

from dynamics import asteroid
from point_cloud import wavefront


class TestDistanceToEdgesCubeOutsideFixedSingle():

    pt_out = np.array([1, 0.5, 0.5])
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')

    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_edges(pt_out, v, f,
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

    D, P, V, E, F = wavefront.distance_to_edges(pt_out, v, f,
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

class TestDistanceToEdgesCubeInside():

    pt_out = np.array([0.4, 0.4, 0.2])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_edges(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = -0.1 * np.sqrt(2) * np.ones_like(D)
    P_exp = np.array([[0.5, 0.5, 0.2],
                      [0.5, 0.5, 0.2]])
    V_exp = np.array([6, 7])
    E_exp = np.array([[6, 7], 
                     [7, 6]])
    F_exp = np.array([[4, 6],
                      [4, 6]])

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)

    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)

    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

    def test_edge(self):
        for E, E_exp in zip(self.E, self.E_exp):
            np.testing.assert_allclose(E, E_exp)

    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)

class TestDistanceToEdgesCubeOutsideEdge():

    pt_out = np.array([0.6, 0.6, 0.6])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')

    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_edges(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = P_exp = V_exp = E_exp = F_exp = []

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)

    def test_point(self):
        np.testing.assert_allclose(self.P, np.empty(shape=(0, 3)))

    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

    def test_edge(self):
        np.testing.assert_allclose(self.E, np.empty(shape=(0,2)))

    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)

class TestDistanceToEdgesCubeSurfaceMultiple():

    pt_out=np.array([0.5, 0.2, 0])
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_edges(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp=0.1 * np.sqrt(2)
    P_exp=np.array([[0.5, 0.1, 0.1],
                    [0.5, 0.1, 0.1]])
    V_exp=np.array([4, 7])
    E_exp = np.array([[7, 4],
                      [4, 7]])
    F_exp=np.array([[6, 7],
                    [6, 7]])
    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)
    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)
    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)
    def test_edge(self):
        for E, E_exp in zip(self.E, self.E_exp):
            np.testing.assert_allclose(E, E_exp)
    def test_face(self):
        for F, F_exp in zip(self.F, self.F_exp):
            np.testing.assert_allclose(F, F_exp)


class TestDistanceToEdgesCubeSurfaceSingle():

    pt_out=np.array([0.5, 0.4, 0.3])
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')

    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_edges(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp=0.1/2*np.sqrt(2)
    P_exp=np.array([0.5, 0.35, 0.35])
    V_exp=np.array([4, 7])
    E_exp = np.array([4, 7])
    F_exp=np.array([6, 7])

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)
    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)
    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)
    def test_edge(self):
        np.testing.assert_allclose(self.E, self.E_exp)
    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)

class TestDistanceToEdgesCubeInsideRandom():
    """Ranom location that is always outside the cube
    """
    pt_out=np.random.uniform(-0.4, 0.4, 3)
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_edges(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)

    def test_distance(self):
        np.testing.assert_allclose(np.isfinite(self.D), True)
    def test_point(self):
        np.testing.assert_allclose(len(self.P) >= 1, True)
    def test_vertex(self):
        np.testing.assert_allclose(np.isfinite(self.V), True)
    def test_edge(self):
        np.testing.assert_allclose(np.isfinite(self.E), True)
    def test_face(self):
        np.testing.assert_allclose(len(self.F) >= 1, True)

class TestDistanceToEdgesCubeOutsideRandom():
    """Ranom location that is always outside the cube
    """
    pt_out=np.array([1, np.random.uniform(-0.5, 0.5), 0])
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_edges(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    def test_distance(self):
        np.testing.assert_allclose(np.isfinite(self.D), True)
    def test_point(self):
        np.testing.assert_allclose(len(self.P) > 1, True)
    def test_vertex(self):
        np.testing.assert_allclose(np.isfinite(self.V), True)
    def test_edge(self):
        np.testing.assert_allclose(np.isfinite(self.E), True)
    def test_face(self):
        np.testing.assert_allclose(len(self.F) >= 1, True)
