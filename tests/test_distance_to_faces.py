import pdb

import numpy as np
from kinematics import sphere

from dynamics import asteroid
from point_cloud import wavefront


class TestDistanceToFacesCubeOutsideFixedSingle():

    pt_out = np.array([1, 0.1, 0])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_faces(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = 0.5
    P_exp = np.array([0.5, 0.1, 0])
    V_exp = [4, 6, 7]
    E_exp = np.array([[6, 4],
                      [7, 6],
                      [4, 7]])
    F_exp = [6]

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

class TestDistanceToFacesCubeOutsideFixedMultiple():
    """This point is equally close to an entire face of the cube

    So there are 4 vertices equidistant from pt
    """
    pt_out = np.array([1, 0, 0])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_faces(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)

    D_exp = np.ones_like(D) * 0.5 
    P_exp = np.array([[0.5, 0, 0],
                      [0.5, 0, 0]])
    V_exp = np.array([[4, 6, 7],
                      [4, 7, 5]])
    E_exp = np.array([[[6, 4], [7, 6], [4, 7]], [[7, 4], [5, 7], [4, 5]]])
    F_exp = np.array([6, 7])

    def test_distance(self):
        np.testing.assert_allclose(np.absolute(self.D), self.D_exp)

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

class TestDistanceToFacesCubeInsideFixedSingle():

    pt_out = np.array([0.4, 0.1, 0])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_faces(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = -0.1
    P_exp = np.array([0.5, 0.1, 0])
    V_exp = np.array([4, 6, 7])
    E_exp = np.array([[6, 4],
                      [7, 6],
                      [4, 7]])
    F_exp = np.array(6)

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

class TestDistanceToFacesCubeInsideMultiple():

    pt_out = np.array([0.4, 0, 0])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_faces(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = np.array([-0.1, -0.1])
    P_exp = np.array([[0.5, 0, 0],
                      [0.5, 0, 0]])
    V_exp = np.array([[4, 6, 7],
                      [4, 7, 5]])
    E_exp = np.array([[[6, 4], [7, 6], [4, 7]],[[7, 4], [5, 7], [4, 5]]])
    F_exp = np.array([6, 7])
    
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
            
class TestDistanceToFacesCubeSurfaceSingle():

    pt_out=np.array([0.51, -0.1, 0.0])
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_faces(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = np.array(0.01)
    P_exp = np.array([0.5, -0.1, 0])
    V_exp = np.array([4, 7, 5])
    E_exp = np.array([[7, 4],
                      [5, 7],
                      [4, 5]])
    F_exp = np.array(7)

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

class TestDistanceToFacesCubeInsideRandom():
    """Ranom location that is always outside the cube
    """
    pt_out=np.array([0.4, np.random.uniform(-0.4, 0.4), 0])
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_faces(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    def test_distance(self):
        np.testing.assert_allclose(np.isfinite(self.D), True)
    def test_point(self):
        np.testing.assert_allclose(len(self.P) >= 3, True)
    def test_vertex(self):
        np.testing.assert_allclose(np.isfinite(self.V), True)
    def test_edge(self):
        np.testing.assert_allclose(np.isfinite(self.E), True)
    def test_face(self):
        np.testing.assert_allclose(self.F.size >= 1, True)

class TestDistanceToFacesCubeOutsideRandom():
    """Ranom location that is always outside the cube
    """
    pt_out=np.array([1, np.random.uniform(-0.4, 0.4), 0])
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_faces(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    def test_distance(self):
        np.testing.assert_allclose(np.isfinite(self.D), True)
    def test_point(self):
        np.testing.assert_allclose(len(self.P) >= 3, True)
    def test_vertex(self):
        np.testing.assert_allclose(np.isfinite(self.V), True)
    def test_edge(self):
        np.testing.assert_allclose(np.isfinite(self.E), True)
    def test_face(self):
        np.testing.assert_allclose(self.F.size >= 1, True)
