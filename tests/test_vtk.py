import vtk
import numpy as np
from point_cloud import wavefront

def test_vtk_major_version():
    np.testing.assert_allclose(vtk.VTK_MAJOR_VERSION, 8)

class TestVTKReadingOBJFilesItokawaLow():
    filename = './data/shape_model/ITOKAWA/itokawa_low.obj'
    vtkPolyData = wavefront.read_obj_to_polydata(filename)
    vtk_vertices, vtk_faces = wavefront.polydatatomesh(vtkPolyData)
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
    vtk_vertices, vtk_faces = wavefront.polydatatomesh(vtkPolyData)
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

#TODO: Tests to make sure we can write to OBJ files correctly. Verify VTK and Wavefront are writing and then reading the same  thing?
