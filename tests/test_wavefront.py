import vtk
import numpy as np
from point_cloud import wavefront
import scipy.io
import pdb

def test_vtk_major_version():
    np.testing.assert_allclose(vtk.VTK_MAJOR_VERSION, 7)

class TestVTKReadingOBJFilesItokawaLow():
    filename = './data/shape_model/ITOKAWA/itokawa_low.obj'
    vtkPolyData = wavefront.read_obj_to_polydata(filename)
    vtk_vertices, vtk_faces = wavefront.vtk_poly_to_numpy(vtkPolyData)
    wavefront_verts, wavefront_faces = wavefront.read_obj(filename)

    def test_numpy_vertices_equal(self):
        """Convert both to numpy representation and compare
        """
        np.testing.assert_allclose(self.vtk_vertices, self.wavefront_verts)

    def test_numpy_faces_equal(self):
        """Convert both to polydata representation and compare
        """
        np.testing.assert_allclose(self.vtk_faces, self.wavefront_faces)

    def test_number_of_faces_equal(self):
        np.testing.assert_allclose(self.vtkPolyData.GetNumberOfPolys(), self.wavefront_faces.shape[0])

    def test_number_of_vertices_equal(self):
        np.testing.assert_allclose(self.vtkPolyData.GetNumberOfPoints(), self.wavefront_verts.shape[0])

class TestVTKReadingOBJFilesItokawaHigh():
    filename = './data/shape_model/ITOKAWA/itokawa_high.obj'
    vtkPolyData = wavefront.read_obj_to_polydata(filename)
    vtk_vertices, vtk_faces = wavefront.vtk_poly_to_numpy(vtkPolyData)
    wavefront_verts, wavefront_faces = wavefront.read_obj(filename)

    def test_numpy_vertices_equal(self):
        """Convert both to numpy representation and compare
        """
        np.testing.assert_allclose(self.vtk_vertices, self.wavefront_verts)

    def test_numpy_faces_equal(self):
        """Convert both to polydata representation and compare
        """
        np.testing.assert_allclose(self.vtk_faces, self.wavefront_faces)

    def test_number_of_faces_equal(self):
        np.testing.assert_allclose(self.vtkPolyData.GetNumberOfPolys(), self.wavefront_faces.shape[0])

    def test_number_of_vertices_equal(self):
        np.testing.assert_allclose(self.vtkPolyData.GetNumberOfPoints(), self.wavefront_verts.shape[0])

class TestCompareVTKAndWavefrontWritingOBJ():
    """Here we test that we can write to OBJ files using both VTK and Wavefront modules
    """
    filename = './data/shape_model/ITOKAWA/itokawa_low.obj'

    vtkPolyData = wavefront.read_obj_to_polydata(filename)
    input_vertices, input_faces = wavefront.vtk_poly_to_numpy(vtkPolyData)
    
    # write OBJ file using both VTK and Wavefront
    wavefront.write_vtkPolyData(vtkPolyData, '/tmp/vtk_output')
    wavefront.write_obj(input_vertices, input_faces, '/tmp/wavefront_output.obj')

    # now read our two files
    polydata_vtk_output = wavefront.read_obj_to_polydata('/tmp/vtk_output.obj')
    vtk_output_vertices, vtk_output_faces = wavefront.vtk_poly_to_numpy(polydata_vtk_output)

    polydata_wavefront_output = wavefront.read_obj_to_polydata('/tmp/wavefront_output.obj')
    wavefront_output_vertices, wavefront_output_faces = wavefront.vtk_poly_to_numpy(polydata_wavefront_output)

    def test_number_vertices(self):
        np.testing.assert_allclose(self.vtk_output_vertices.shape,
                                   self.wavefront_output_vertices.shape)
    
    def test_number_of_faces(self):
        np.testing.assert_allclose(self.vtk_output_faces.shape,
                                   self.wavefront_output_faces.shape)

    def test_vertices_equal(self):
        np.testing.assert_allclose(self.vtk_output_vertices,
                                   self.wavefront_output_vertices)

    def test_faces_equal(self):
        np.testing.assert_allclose(self.vtk_output_faces,
                                   self.wavefront_output_faces)

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
