"""Test out the incremental mesh updating
"""
import numpy as np
from point_cloud import wavefront


class TestVertexInsertion():
    v, f = wavefront.read_obj('./integration/cube.obj')
    pt = np.array([1, 1, 1])
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    D, P, V, E, F, primitive = wavefront.distance_to_mesh(
        pt, v, f, mesh_parameters)
    nv, nf = wavefront.vertex_insertion(pt, v, f, D, P, V, E, F)

    # expected solutions
    nv_exp = np.array([[-0.5, -0.5, -0.5],
                       [-0.5, -0.5, 0.5],
                       [-0.5, 0.5, -0.5],
                       [-0.5, 0.5, 0.5],
                       [0.5, -0.5, -0.5],
                       [0.5, -0.5, 0.5],
                       [0.5, 0.5, -0.5],
                       [1., 1., 1.]])

    nf_exp = np.array([[0, 6, 4],
                       [0, 2, 6],
                       [0, 3, 2],
                       [0, 1, 3],
                       [2, 7, 6],
                       [2, 3, 7],
                       [4, 6, 7],
                       [4, 7, 5],
                       [0, 4, 5],
                       [0, 5, 1],
                       [1, 5, 7],
                       [1, 7, 3]])

    def test_last_vertex(self):
        np.testing.assert_allclose(self.nv[-1, :], self.pt)

    def test_vertices(self):
        np.testing.assert_allclose(self.nv, self.nv_exp)

    def test_faces(self):
        np.testing.assert_allclose(self.nf, self.nf_exp)
