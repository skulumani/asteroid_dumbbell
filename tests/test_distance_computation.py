import pdb

import numpy as np
from kinematics import sphere

from dynamics import asteroid
from point_cloud import wavefront


class TestDistanceToVerticesCubeOutsideFixedSingle():

    pt_out = np.array([1, 1, 1])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = 0.5 * np.sqrt(3)
    P_exp = np.array([0.5, 0.5, 0.5])
    V_exp = 7
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
    F_exp = [4, 5, 6, 7, 10, 11]

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


class TestDistanceToVerticesCubeOutsideFixedMultiple():
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

    D, P, V, E, F = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)

    D_exp = np.ones_like(D) * 0.5 * np.sqrt(3)
    P_exp = np.array([[0.5, -0.5, -0.5],
                      [0.5, -0.5,  0.5],
                      [0.5,  0.5, -0.5],
                      [0.5,  0.5,  0.5]])
    V_exp = np.array([4, 5, 6, 7])
    E_exp = np.array([np.array([[6, 4],
                                [7, 4],
                                [4, 0],
                                [4, 6],
                                [5, 4],
                                [0, 4],
                                [4, 7],
                                [4, 5]]),
                      np.array([[5, 0],
                                [5, 1],
                                [5, 7],
                                [5, 4],
                                [1, 5],
                                [7, 5],
                                [4, 5],
                                [0, 5]]),
                      np.array([[6, 0],
                                [6, 4],
                                [4, 6],
                                [6, 2],
                                [6, 7],
                                [7, 6],
                                [0, 6],
                                [2, 6]]),
                      np.array([[7, 2],
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
                                [1, 7]])])
    F_exp = np.array([list([0, 6, 7, 8]),
                      list([7, 8, 9, 10]),
                      list([0, 1, 4, 6]),
                      list([4, 5, 6, 7, 10, 11])])

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


class TestDistanceToVerticesCubeInsideFixedSingle():

    pt_out = np.array([0.4, 0.4, 0.4])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = 0.4 * np.sqrt(3) - 0.5 * np.sqrt(3)
    P_exp = np.array([0.5, 0.5, 0.5])
    V_exp = 7
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
    F_exp = [4, 5, 6, 7, 10, 11]

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


class TestDistanceToVerticesCubeInsideMultiple():

    pt_out = np.array([0, 0, 0])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp = np.array([-0.8660254, -0.8660254, -0.8660254, -0.8660254,
                     -0.8660254, -0.8660254, -0.8660254, -0.8660254])
    P_exp = v_cube
    V_exp = np.arange(0, 8)
    E_exp = np.array([np.array([[6, 0],
				[2, 0],
				[3, 0],
				[1, 0],
				[4, 0],
				[5, 0],
				[0, 4],
				[0, 6],
				[0, 2],
				[0, 3],
				[0, 5],
				[0, 1]]),
		      np.array([[1, 0],
		  [5, 1],
		  [7, 1],
		  [3, 1],
		  [1, 5],
		  [0, 1],
		  [1, 7],
		  [1, 3]]),
		      np.array([[2, 0],
		  [7, 2],
		  [3, 2],
		  [6, 2],
		  [2, 3],
		  [0, 2],
		  [2, 6],
		  [2, 7]]),
		      np.array([[3, 0],
		  [3, 2],
		  [2, 3],
		  [3, 1],
		  [7, 3],
		  [3, 7],
		  [0, 3],
		  [1, 3]]),
		      np.array([[6, 4],
		  [7, 4],
		  [4, 0],
		  [4, 6],
		  [5, 4],
		  [0, 4],
		  [4, 7],
		  [4, 5]]),
		      np.array([[5, 0],
		  [5, 1],
		  [5, 7],
		  [5, 4],
		  [1, 5],
		  [7, 5],
		  [4, 5],
		  [0, 5]]),
		      np.array([[6, 0],
		  [6, 4],
		  [4, 6],
		  [6, 2],
		  [6, 7],
		  [7, 6],
		  [0, 6],
		  [2, 6]]),
		      np.array([[7, 2],
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
		  [1, 7]])], dtype=object)
    F_exp = np.array([list([0, 1, 2, 3, 8, 9]), list([3, 9, 10, 11]), list([1, 2, 4, 5]),
		      list([2, 3, 5, 11]), list([0, 6, 7, 8]), list([7, 8, 9, 10]),
		      list([0, 1, 4, 6]), list([4, 5, 6, 7, 10, 11])], dtype=object)

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

class TestDistanceToVerticesCubeSurfaceSingle():

    pt_out=np.array([0.5, 0.4, 0.4])
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp=-0.1 * np.sqrt(2)
    P_exp=np.array([0.5, 0.5, 0.5])
    V_exp=7
    E_exp=np.array([[7, 2],
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
    F_exp=[4, 5, 6, 7, 10, 11]

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

class TestDistanceToVerticesCubeOutsideSurfaceMultiple():
    """This point is actually on the surface of the cube

    So there are 4 vertices equidistant from pt
    """
    pt_out=np.array([0.5, 0, 0])
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    D_exp=-np.ones_like(D) * 0.5 * np.sqrt(2)
    P_exp=np.array([[0.5, -0.5, -0.5],
                      [0.5, -0.5,  0.5],
                      [0.5,  0.5, -0.5],
                      [0.5,  0.5,  0.5]])
    V_exp=np.array([4, 5, 6, 7])
    E_exp=np.array([np.array([[6, 4],
        [7, 4],
        [4, 0],
        [4, 6],
        [5, 4],
        [0, 4],
        [4, 7],
        [4, 5]]),
        np.array([[5, 0],
        [5, 1],
        [5, 7],
        [5, 4],
        [1, 5],
        [7, 5],
        [4, 5],
        [0, 5]]),
        np.array([[6, 0],
        [6, 4],
        [4, 6],
        [6, 2],
        [6, 7],
        [7, 6],
        [0, 6],
        [2, 6]]),
        np.array([[7, 2],
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
        [1, 7]])], dtype = object)
    F_exp=np.array([list([0, 6, 7, 8]),
                      list([7, 8, 9, 10]),
                      list([0, 1, 4, 6]),
                      list([4, 5, 6, 7, 10, 11])])

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

class TestDistanceToVerticesCubeInsideRandom():
    """Ranom location that is always outside the cube
    """
    pt_out=np.random.uniform(0.01, 0.49 * np.sqrt(3)) * sphere.rand(2)
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
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
        np.testing.assert_allclose(len(self.F) >= 1, True)

class TestDistanceToVerticesCubeOutsideRandom():
    """Ranom location that is always outside the cube
    """
    pt_out=np.random.uniform(0.51 * np.sqrt(3), 1) * sphere.rand(2)
    v_cube, f_cube=wavefront.read_obj('./integration/cube.obj')
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    ast = ast.loadmesh(v_cube, f_cube, 'cube')
    edge_vertex_map=ast.asteroid_grav['edge_vertex_map']
    edge_face_map=ast.asteroid_grav['edge_face_map']
    normal_face=ast.asteroid_grav['normal_face']
    vf_map=ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F=wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
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
        np.testing.assert_allclose(len(self.F) >= 1, True)

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
