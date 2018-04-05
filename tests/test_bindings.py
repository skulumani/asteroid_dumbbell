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
