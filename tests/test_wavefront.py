"""Test out the Wavefront reading and writing
"""

from point_cloud import wavefront
import numpy as np
import scipy.io
import pdb



# TODO: Compare with the matlab file with faces/vertices

def test_reader_matches():
    """Compare faces/verts to Matlab mat file?"""
    pass


# TODO: Test to convert OBJ to VTK to Numpy and make sure the numbers are correct

class TestItokawaHighReadingAndWriting():
    filename = './data/shape_model/ITOKAWA/itokawa_high.obj'
    num_faces = 786432
    num_verts =396294
    num_edges = 1182724

    verts, faces = wavefront.read_obj(filename)

    wavefront.write_obj(verts, faces, '/tmp/test.obj')       
    out_verts, out_faces = wavefront.read_obj('/tmp/test.obj')
    def test_num_edges(self):
        edges_obj = self.verts.shape[0] + self.faces.shape[0] - 2
        np.testing.assert_allclose(edges_obj, self.num_edges)

    def test_num_faces(self):
        np.testing.assert_allclose(self.faces.shape, (self.num_faces, 3))

    def test_num_vertices(self):
        np.testing.assert_allclose(self.verts.shape, (self.num_verts, 3))

    def test_output_vertices_equal(self):
        """Write verts/faces to a different file and make sure it's the same
        """
        np.testing.assert_allclose(self.out_verts, self.verts)

    def test_output_faces_equal(self):
        np.testing.assert_allclose(self.out_faces, self.faces)

class TestItokawaLowReadingAndWriting():
    filename = './data/shape_model/ITOKAWA/itokawa_low.obj'
    num_faces = 49152
    num_verts = 25350
    num_edges = 74500

    verts, faces = wavefront.read_obj(filename)

    wavefront.write_obj(verts, faces, '/tmp/test.obj')       
    out_verts, out_faces = wavefront.read_obj('/tmp/test.obj')
    def test_num_edges(self):
        edges_obj = self.verts.shape[0] + self.faces.shape[0] - 2
        np.testing.assert_allclose(edges_obj, self.num_edges)

    def test_num_faces(self):
        np.testing.assert_allclose(self.faces.shape, (self.num_faces, 3))

    def test_num_vertices(self):
        np.testing.assert_allclose(self.verts.shape, (self.num_verts, 3))

    def test_output_vertices_equal(self):
        """Write verts/faces to a different file and make sure it's the same
        """
        np.testing.assert_allclose(self.out_verts, self.verts)

    def test_output_faces_equal(self):
        np.testing.assert_allclose(self.out_faces, self.faces)
