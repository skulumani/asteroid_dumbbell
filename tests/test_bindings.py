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
polyhedron_potential = pytest.importorskip('lib.polyhedron_potential')
surface_mesh = pytest.importorskip('lib.surface_mesh')
reconstruct = pytest.importorskip('lib.reconstruct')

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
        intersections = np.array([0, 0, 0], dtype=np.float64)
        flag = self.caster.castray(self.pt, np.array([0, 0, 0], dtype=np.float64),
                                   intersections)
    
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
        min_index = 0
        max_index = 10
        np.testing.assert_allclose(vp[min_index:max_index, :], rmesh.get_verts()[min_index:max_index, :])
