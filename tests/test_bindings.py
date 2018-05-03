"""Test out all the C++ bindings and operations

Extended description of the module

Notes
-----
These tests need the C++ bindings to be built first. 
You can run cmake in a build directory and everythign should work as expected

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""

import numpy as np
import pytest

from point_cloud import wavefront
cgal = pytest.importorskip('lib.cgal')
mesh_data = pytest.importorskip('lib.mesh_data')
asteroid = pytest.importorskip('lib.asteroid')
surface_mesh = pytest.importorskip('lib.surface_mesh')
reconstruct = pytest.importorskip('lib.reconstruct')
geodesic = pytest.importorskip('lib.geodesic')

class TestMeshData:
    v, f = wavefront.read_obj('./integration/cube.obj')
    mesh = mesh_data.MeshData(v, f)
    
    def test_vertices(self):
        np.testing.assert_allclose(self.mesh.get_verts(), self.v)

    def test_faces(self):
        np.testing.assert_allclose(self.mesh.get_faces(), self.f)

    def test_update_mesh(self):
        v, f = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj')
        self.mesh.update_mesh(v, f)
        np.testing.assert_allclose(self.mesh.get_verts(), v)
        np.testing.assert_allclose(self.mesh.get_faces(), f)

class TestSurfaceMesher:
    smesh = surface_mesh.SurfMesh(1, 1, 1, 10, 0.2, 0.5)

    def test_vertices(self):
        np.testing.assert_allclose(self.smesh.verts().shape[1], 3)

    def test_faces(self):
        np.testing.assert_allclose(self.smesh.faces().shape[1], 3)

class TestMeshDist:
    
    # load the vertices, faces
    v, f = wavefront.read_obj('./integration/cube.obj') 
    # create a mesh
    mesh = mesh_data.MeshData(v, f)
    mesh_dist = cgal.MeshDistance(mesh)
    caster = cgal.RayCaster(mesh)

    pt = np.array([2, 0, 0], dtype=np.float64)

    # pass to distance function
    def test_minimum_distance(self):
        dist = self.caster.minimum_distance(self.pt)
        np.testing.assert_allclose(dist, 1.5)
    
    def test_intersection_raycasting(self):
        intersections = self.caster.castray(self.pt, np.array([0, 0, 0],
                                                              dtype=np.float64))
    
    # also test out the ray caster
 
class TestReconstructMeshCube:
    
    # load the vertices, faces
    v, f = wavefront.read_obj('./integration/cube.obj') 
    # create a mesh
    mesh = mesh_data.MeshData(v, f)
    pt = np.array([1, 1, 1])
    w = np.full(v.shape[0], 1)
    max_angle = 0.5

    def test_mesh_constructor(self):
        rmesh = reconstruct.ReconstructMesh(self.mesh)
        np.testing.assert_allclose(rmesh.get_verts(), self.v)
        np.testing.assert_allclose(rmesh.get_faces(), self.f)

    def test_array_constructor(self):
        rmesh = reconstruct.ReconstructMesh(self.v, self.f, self.w)
        np.testing.assert_allclose(rmesh.get_verts(), self.v)
        np.testing.assert_allclose(rmesh.get_faces(), self.f)

    def test_update_matches_python(self):
        vp, wp = wavefront.spherical_incremental_mesh_update(self.pt, self.v, self.f, self.w, self.max_angle)
        rmesh = reconstruct.ReconstructMesh(self.v, self.f, self.w)
        rmesh.update(self.pt, self.max_angle)

        np.testing.assert_allclose(vp, rmesh.get_verts())
        np.testing.assert_allclose(wp, np.squeeze(rmesh.get_weights()))

class TestReconstructMeshCastalia:
    
    # load the vertices, faces
    v, f = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj') 
    # create a mesh
    mesh = mesh_data.MeshData(v, f)
    pt = np.array([1, 1, 1])
    w = np.full(v.shape[0], 1)
    max_angle = 0.5

    def test_mesh_constructor(self):
        rmesh = reconstruct.ReconstructMesh(self.mesh)
        np.testing.assert_allclose(rmesh.get_verts(), self.v)
        np.testing.assert_allclose(rmesh.get_faces(), self.f)

    def test_array_constructor(self):
        rmesh = reconstruct.ReconstructMesh(self.v, self.f, self.w)
        np.testing.assert_allclose(rmesh.get_verts(), self.v)
        np.testing.assert_allclose(rmesh.get_faces(), self.f)

    def test_update_matches_python(self):
        vp, wp = wavefront.spherical_incremental_mesh_update(self.pt, self.v, self.f, self.w, self.max_angle)
        rmesh = reconstruct.ReconstructMesh(self.v, self.f, self.w)
        rmesh.update(self.pt, self.max_angle)
        np.testing.assert_allclose(vp, rmesh.get_verts())
        # np.testing.assert_allclose(wp, np.squeeze(rmesh.get_weights()))


class TestGeodesic:
    # generate some random unit vectors
    pt = np.random.rand(3)
    pt = pt / np.linalg.norm(pt)

    v, _ = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_high.obj')

    def test_central_angle(self):
        csigma = geodesic.central_angle(self.pt, self.v)
        psigma = wavefront.spherical_distance(self.pt, self.v)
        np.testing.assert_allclose(csigma, psigma)

class TestAsteroid:
    v, f = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj')
    state = np.array([1, 2, 3])

    def test_mesh_param(self):
        mesh_param = asteroid.MeshParam(self.v, self.f);
        np.testing.assert_allclose(1, 1);

    def test_asteroid_potential(self):
        mesh_param = asteroid.MeshParam(self.v, self.f);
        ast = asteroid.Asteroid('castalia', mesh_param)
        ast.polyhedron_potential(self.state)
        np.testing.assert_allclose(ast.get_potential(), 2.4923324e-08, 1e-4)

    def test_ensure_asteroid_potential_matching(self):
        # some random numbers for the state
        from dynamics import asteroid as asteroid_python

        ast_python = asteroid_python.Asteroid('castalia', 0, 'obj')

        mesh_param = asteroid.MeshParam(self.v, self.f)
        ast_cpp = asteroid.Asteroid('castalia', mesh_param)
    
        for ii in range(10):
            x = np.random.rand()
            state = np.array([x, 1, 1])

            Up, _ , _ ,_ = ast_python.polyhedron_potential(state)
            ast_cpp.polyhedron_potential(state)

            np.testing.assert_allclose(ast_cpp.get_potential(), Up, 1e-6)


