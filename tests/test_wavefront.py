import pdb

import numpy as np
import vtk
import scipy.io
from kinematics import sphere

from point_cloud import wavefront


def test_vtk_major_version():
    np.testing.assert_allclose(vtk.VTK_MAJOR_VERSION, 7)

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

class TestCompareVTKAndWavefrontWritingOBJ():
    """Here we test that we can write to OBJ files using both VTK and Wavefront modules
    """
    filename = './data/shape_model/ITOKAWA/itokawa_low.obj'

    vtkPolyData = wavefront.read_obj_to_polydata(filename)
    input_vertices, input_faces = wavefront.polydatatomesh(vtkPolyData)
    
    # write OBJ file using both VTK and Wavefront
    wavefront.write_vtkPolyData(vtkPolyData, '/tmp/vtk_output')
    wavefront.write_obj(input_vertices, input_faces, '/tmp/wavefront_output.obj')

    # now read our two files
    polydata_vtk_output = wavefront.read_obj_to_polydata('/tmp/vtk_output.obj')
    vtk_output_vertices, vtk_output_faces = wavefront.polydatatomesh(polydata_vtk_output)

    polydata_wavefront_output = wavefront.read_obj_to_polydata('/tmp/wavefront_output.obj')
    wavefront_output_vertices, wavefront_output_faces = wavefront.polydatatomesh(polydata_wavefront_output)

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

def test_reader_matches():
    """Compare faces/verts to Matlab mat file?"""
    pass


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

class TestDecimation():
    """Ensure decimation is working properly
    """
    filename = './data/shape_model/ITOKAWA/itokawa_low.obj'
    ratio = 0.5
    v, f = wavefront.read_obj(filename)
    dv, df = wavefront.decimate_numpy(v, f, ratio)
    
    def test_ensure_vertices_ratio(self):
        np.testing.assert_array_less(self.dv.shape[0], self.v.shape[0])

    def test_ensure_faces_ratio(self):
        np.testing.assert_array_less(self.df.shape[0], self.f.shape[0])

    def test_ensure_polyhedron_is_closed(self):
        pass

class TestReconstruction():
    filename = './data/shape_model/ITOKAWA/itokawa_low.obj'
    ratio = 0.5
    v, f = wavefront.read_obj(filename)
    dv, df = wavefront.decimate_numpy(v, f, ratio)
    
    rv, rf = wavefront.reconstruct_numpy(v)

    def test_reconstruct_vertices(self):
        np.testing.assert_allclose(self.v.shape[1], self.rv.shape[1])

    def test_reconstruct_faces(self):
        np.testing.assert_allclose(self.f.shape[1], self.rf.shape[1])

    
class TestReadingCubeOBJ():
    """Read a simple cube and run some tests"""
    v, f = wavefront.read_obj('./integration/cube.obj')

    (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3,
     e1_vertex_map, e2_vertex_map, e3_vertex_map, 
     normal_face, e1_normal, e2_normal,e3_normal, center_face,
     e_vertex_map, unique_index) = wavefront.polyhedron_parameters(v, f)

    num_v = v.shape[0]
    num_f = f.shape[0]
    num_e = 3 * (num_v - 2)
    def test_number_faces(self):
        np.testing.assert_allclose(self.f.shape, (self.num_f,3))
    
    def test_number_e1_edges(self):
        np.testing.assert_allclose(self.e1.shape, (self.num_f, 3))
    def test_number_e2_edges(self):
        np.testing.assert_allclose(self.e2.shape, (self.num_f, 3))
    def test_number_e3_edges(self):
        np.testing.assert_allclose(self.e3.shape, (self.num_f, 3))
    
    def test_number_vertices(self):
        np.testing.assert_allclose(self.v.shape, (self.num_v, 3))

    def test_Fa(self):
        np.testing.assert_allclose(self.Fa, self.f[:, 0])
    def test_Fb(self):
        np.testing.assert_allclose(self.Fb, self.f[:, 1])
    def test_Fc(self):
        np.testing.assert_allclose(self.Fc, self.f[:, 2])

    def test_V1_shape(self):
        np.testing.assert_allclose(self.V1.shape, (self.num_f, 3))
    def test_V2_shape(self):
        np.testing.assert_allclose(self.V1.shape, (self.num_f, 3))
    def test_V3_shape(self):
        np.testing.assert_allclose(self.V3.shape, (self.num_f, 3))

    def test_e1_vertex_map_shape(self):
        np.testing.assert_allclose(self.e1_vertex_map.shape, (self.num_f, 2))
    def test_e2_vertex_map_shape(self):
        np.testing.assert_allclose(self.e2_vertex_map.shape, (self.num_f, 2))
    def test_e3_vertex_map_shape(self):
        np.testing.assert_allclose(self.e3_vertex_map.shape, (self.num_f, 2))

    def test_e1_vertex_map_values(self):
        np.testing.assert_allclose(self.e1_vertex_map, np.stack((self.f[:, 1], self.f[:, 0]), axis=1))
    def test_e2_vertex_map_values(self):
        np.testing.assert_allclose(self.e2_vertex_map, np.stack((self.f[:, 2], self.f[:, 1]), axis=1))
    def test_e3_vertex_map_values(self):
        np.testing.assert_allclose(self.e3_vertex_map, np.stack((self.f[:, 0], self.f[:, 2]), axis=1))

    def test_normal_face_shape(self):
        np.testing.assert_allclose(self.normal_face.shape, (self.num_f, 3))
    
    def test_e1_normal_shape(self):
        np.testing.assert_allclose(self.e1_normal.shape, (self.num_f, 3))
    def test_e2_normal_shape(self):
        np.testing.assert_allclose(self.e2_normal.shape, (self.num_f, 3))
    def test_e3_normal_shape(self):
        np.testing.assert_allclose(self.e3_normal.shape, (self.num_f, 3))
    def test_center_face(self):
        np.testing.assert_allclose(self.center_face.shape, (self.num_f, 3))
    def test_normal_face_function(self):
        normal_face = wavefront.normal_face(self.v, self.f)
        np.testing.assert_allclose(normal_face, self.normal_face)
    def test_center_face_function(self):
        cof = wavefront.center_of_face(self.v, self.f)
        np.testing.assert_allclose(cof, self.center_face)

class TestEdgeSearchingItokawaMat32():
    # load mat file
    mat = scipy.io.loadmat('./data/shape_model/ITOKAWA/itokawa_model.mat')
    f = mat['F_32'] - 1
    v = mat['V_32'] 

    (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3, e1_vertex_map, e2_vertex_map,
    e3_vertex_map, normal_face, e1_normal, e2_normal,e3_normal, center_face, e_vertex_map, unique_index) = wavefront.polyhedron_parameters(v, f)

    (e1_ind1b, e1_ind2b, e1_ind3b,
    e2_ind1b, e2_ind2b, e2_ind3b,
    e3_ind1b, e3_ind2b, e3_ind3b) = wavefront.search_edge(e1, e2, e3)

    (e1_ind1b_new, e1_ind2b_new, e1_ind3b_new,
    e2_ind1b_new, e2_ind2b_new, e2_ind3b_new,
    e3_ind1b_new, e3_ind2b_new, e3_ind3b_new) = wavefront.search_edge_vertex_map(e1_vertex_map, e2_vertex_map, e3_vertex_map)
     
    num_v = v.shape[0]
    num_f = f.shape[0]
    num_e = 3 * (num_v - 2)
    
    def test_number_faces(self):
        np.testing.assert_allclose(self.f.shape, (self.num_f,3))

    def test_number_e1_edges(self):
        np.testing.assert_allclose(self.e1.shape, (self.num_f, 3))
    def test_number_e2_edges(self):
        np.testing.assert_allclose(self.e2.shape, (self.num_f, 3))
    def test_number_e3_edges(self):
        np.testing.assert_allclose(self.e3.shape, (self.num_f, 3))
    
    def test_number_vertices(self):
        np.testing.assert_allclose(self.v.shape, (self.num_v, 3))
    
    def test_Fa(self):
        np.testing.assert_allclose(self.Fa, self.f[:, 0])
    def test_Fb(self):
        np.testing.assert_allclose(self.Fb, self.f[:, 1])
    def test_Fc(self):
        np.testing.assert_allclose(self.Fc, self.f[:, 2])
    def test_V1_shape(self):
        np.testing.assert_allclose(self.V1.shape, (self.num_f, 3))
    def test_V2_shape(self):
        np.testing.assert_allclose(self.V1.shape, (self.num_f, 3))
    def test_V3_shape(self):
        np.testing.assert_allclose(self.V3.shape, (self.num_f, 3))
    def test_e1_vertex_map_shape(self):
        np.testing.assert_allclose(self.e1_vertex_map.shape, (self.num_f, 2))
    def test_e2_vertex_map_shape(self):
        np.testing.assert_allclose(self.e2_vertex_map.shape, (self.num_f, 2))
    def test_e3_vertex_map_shape(self):
        np.testing.assert_allclose(self.e3_vertex_map.shape, (self.num_f, 2))

    def test_e1_vertex_map_values(self):
        np.testing.assert_allclose(self.e1_vertex_map, np.stack((self.f[:, 1], self.f[:, 0]), axis=1))
    def test_e2_vertex_map_values(self):
        np.testing.assert_allclose(self.e2_vertex_map, np.stack((self.f[:, 2], self.f[:, 1]), axis=1))
    def test_e3_vertex_map_values(self):
        np.testing.assert_allclose(self.e3_vertex_map, np.stack((self.f[:, 0], self.f[:, 2]), axis=1))

    def test_normal_face_shape(self):
        np.testing.assert_allclose(self.normal_face.shape, (self.num_f, 3))
    
    def test_e1_normal_shape(self):
        np.testing.assert_allclose(self.e1_normal.shape, (self.num_f, 3))
    def test_e2_normal_shape(self):
        np.testing.assert_allclose(self.e2_normal.shape, (self.num_f, 3))
    def test_e3_normal_shape(self):
        np.testing.assert_allclose(self.e3_normal.shape, (self.num_f, 3))
    def test_center_face(self):
        np.testing.assert_allclose(self.center_face.shape, (self.num_f, 3))

    def test_e1_edge_searching_shape(self):
        np.testing.assert_allclose(self.e1_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind3b.shape, (self.num_f,))
    def test_e2_edge_searching_shape(self):
        np.testing.assert_allclose(self.e2_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind3b.shape, (self.num_f,))
    def test_e3_edge_searching_shape(self):
        np.testing.assert_allclose(self.e3_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind3b.shape, (self.num_f,))
    def test_e1_vertex_map_searching(self):
        np.testing.assert_allclose(self.e1_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind3b_new.shape, (self.num_f,))
    def test_e2_vertex_map_searching(self):
        np.testing.assert_allclose(self.e2_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind3b_new.shape, (self.num_f,))
    def test_e3_vertex_map_searching(self):
        np.testing.assert_allclose(self.e3_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind3b_new.shape, (self.num_f,))

    def test_e1_searching_matches(self):
        np.testing.assert_allclose(self.e1_ind1b_new, self.e1_ind1b)
        np.testing.assert_allclose(self.e1_ind2b_new, self.e1_ind2b)
        np.testing.assert_allclose(self.e1_ind3b_new, self.e1_ind3b)
    def test_e2_searching_matches(self):
        np.testing.assert_allclose(self.e2_ind1b_new, self.e2_ind1b)
        np.testing.assert_allclose(self.e2_ind2b_new, self.e2_ind2b)
        np.testing.assert_allclose(self.e2_ind3b_new, self.e2_ind3b)
    def test_e3_searching_matches(self):
        np.testing.assert_allclose(self.e3_ind1b_new, self.e3_ind1b)
        np.testing.assert_allclose(self.e3_ind2b_new, self.e3_ind2b)
        np.testing.assert_allclose(self.e3_ind3b_new, self.e3_ind3b)
    def test_normal_face_function(self):
        normal_face = wavefront.normal_face(self.v, self.f)
        np.testing.assert_allclose(normal_face, self.normal_face)
    def test_center_face_function(self):
        cof = wavefront.center_of_face(self.v, self.f)
        np.testing.assert_allclose(cof, self.center_face)

class TestEdgeSearchingItokawaMat2048():
    # load mat file
    mat = scipy.io.loadmat('./data/shape_model/ITOKAWA/itokawa_model.mat')
    f = mat['F_2048'] - 1
    v = mat['V_2048'] 

    (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3, e1_vertex_map, e2_vertex_map,
    e3_vertex_map, normal_face, e1_normal, e2_normal,e3_normal, center_face, e_vertex_map, unique_index) = wavefront.polyhedron_parameters(v, f)

    (e1_ind1b, e1_ind2b, e1_ind3b,
    e2_ind1b, e2_ind2b, e2_ind3b,
    e3_ind1b, e3_ind2b, e3_ind3b) = wavefront.search_edge(e1, e2, e3)

    (e1_ind1b_new, e1_ind2b_new, e1_ind3b_new,
    e2_ind1b_new, e2_ind2b_new, e2_ind3b_new,
    e3_ind1b_new, e3_ind2b_new, e3_ind3b_new) = wavefront.search_edge_vertex_map(e1_vertex_map, e2_vertex_map, e3_vertex_map)
     
    num_v = v.shape[0]
    num_f = f.shape[0]
    num_e = 3 * (num_v - 2)
    
    def test_number_faces(self):
        np.testing.assert_allclose(self.f.shape, (self.num_f,3))

    def test_number_e1_edges(self):
        np.testing.assert_allclose(self.e1.shape, (self.num_f, 3))
    def test_number_e2_edges(self):
        np.testing.assert_allclose(self.e2.shape, (self.num_f, 3))
    def test_number_e3_edges(self):
        np.testing.assert_allclose(self.e3.shape, (self.num_f, 3))
    
    def test_number_vertices(self):
        np.testing.assert_allclose(self.v.shape, (self.num_v, 3))
    
    def test_Fa(self):
        np.testing.assert_allclose(self.Fa, self.f[:, 0])
    def test_Fb(self):
        np.testing.assert_allclose(self.Fb, self.f[:, 1])
    def test_Fc(self):
        np.testing.assert_allclose(self.Fc, self.f[:, 2])
    def test_V1_shape(self):
        np.testing.assert_allclose(self.V1.shape, (self.num_f, 3))
    def test_V2_shape(self):
        np.testing.assert_allclose(self.V1.shape, (self.num_f, 3))
    def test_V3_shape(self):
        np.testing.assert_allclose(self.V3.shape, (self.num_f, 3))
    def test_e1_vertex_map_shape(self):
        np.testing.assert_allclose(self.e1_vertex_map.shape, (self.num_f, 2))
    def test_e2_vertex_map_shape(self):
        np.testing.assert_allclose(self.e2_vertex_map.shape, (self.num_f, 2))
    def test_e3_vertex_map_shape(self):
        np.testing.assert_allclose(self.e3_vertex_map.shape, (self.num_f, 2))

    def test_e1_vertex_map_values(self):
        np.testing.assert_allclose(self.e1_vertex_map, np.stack((self.f[:, 1], self.f[:, 0]), axis=1))
    def test_e2_vertex_map_values(self):
        np.testing.assert_allclose(self.e2_vertex_map, np.stack((self.f[:, 2], self.f[:, 1]), axis=1))
    def test_e3_vertex_map_values(self):
        np.testing.assert_allclose(self.e3_vertex_map, np.stack((self.f[:, 0], self.f[:, 2]), axis=1))

    def test_normal_face_shape(self):
        np.testing.assert_allclose(self.normal_face.shape, (self.num_f, 3))
    
    def test_e1_normal_shape(self):
        np.testing.assert_allclose(self.e1_normal.shape, (self.num_f, 3))
    def test_e2_normal_shape(self):
        np.testing.assert_allclose(self.e2_normal.shape, (self.num_f, 3))
    def test_e3_normal_shape(self):
        np.testing.assert_allclose(self.e3_normal.shape, (self.num_f, 3))
    def test_center_face(self):
        np.testing.assert_allclose(self.center_face.shape, (self.num_f, 3))

    def test_e1_edge_searching_shape(self):
        np.testing.assert_allclose(self.e1_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind3b.shape, (self.num_f,))
    def test_e2_edge_searching_shape(self):
        np.testing.assert_allclose(self.e2_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind3b.shape, (self.num_f,))
    def test_e3_edge_searching_shape(self):
        np.testing.assert_allclose(self.e3_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind3b.shape, (self.num_f,))
    def test_e1_vertex_map_searching(self):
        np.testing.assert_allclose(self.e1_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind3b_new.shape, (self.num_f,))
    def test_e2_vertex_map_searching(self):
        np.testing.assert_allclose(self.e2_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind3b_new.shape, (self.num_f,))
    def test_e3_vertex_map_searching(self):
        np.testing.assert_allclose(self.e3_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind3b_new.shape, (self.num_f,))

    def test_e1_searching_matches(self):
        np.testing.assert_allclose(self.e1_ind1b_new, self.e1_ind1b)
        np.testing.assert_allclose(self.e1_ind2b_new, self.e1_ind2b)
        np.testing.assert_allclose(self.e1_ind3b_new, self.e1_ind3b)
    def test_e2_searching_matches(self):
        np.testing.assert_allclose(self.e2_ind1b_new, self.e2_ind1b)
        np.testing.assert_allclose(self.e2_ind2b_new, self.e2_ind2b)
        np.testing.assert_allclose(self.e2_ind3b_new, self.e2_ind3b)
    def test_e3_searching_matches(self):
        np.testing.assert_allclose(self.e3_ind1b_new, self.e3_ind1b)
        np.testing.assert_allclose(self.e3_ind2b_new, self.e3_ind2b)
        np.testing.assert_allclose(self.e3_ind3b_new, self.e3_ind3b)
    def test_normal_face_function(self):
        normal_face = wavefront.normal_face(self.v, self.f)
        np.testing.assert_allclose(normal_face, self.normal_face)
    def test_center_face_function(self):
        cof = wavefront.center_of_face(self.v, self.f)
        np.testing.assert_allclose(cof, self.center_face)

class TestEdgeSearchingCastaliaMat4092():
    # load mat file
    mat = scipy.io.loadmat('./data/shape_model/CASTALIA/castalia_model.mat')
    f = mat['F_4092'] - 1
    v = mat['V_4092'] 

    (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3, e1_vertex_map, e2_vertex_map,
    e3_vertex_map, normal_face, e1_normal, e2_normal,e3_normal, center_face, e_vertex_map, unique_index) = wavefront.polyhedron_parameters(v, f)

    (e1_ind1b, e1_ind2b, e1_ind3b,
    e2_ind1b, e2_ind2b, e2_ind3b,
    e3_ind1b, e3_ind2b, e3_ind3b) = wavefront.search_edge(e1, e2, e3)

    (e1_ind1b_new, e1_ind2b_new, e1_ind3b_new,
    e2_ind1b_new, e2_ind2b_new, e2_ind3b_new,
    e3_ind1b_new, e3_ind2b_new, e3_ind3b_new) = wavefront.search_edge_vertex_map(e1_vertex_map, e2_vertex_map, e3_vertex_map)
     
    num_v = v.shape[0]
    num_f = f.shape[0]
    num_e = 3 * (num_v - 2)
    
    def test_number_faces(self):
        np.testing.assert_allclose(self.f.shape, (self.num_f,3))

    def test_number_e1_edges(self):
        np.testing.assert_allclose(self.e1.shape, (self.num_f, 3))
    def test_number_e2_edges(self):
        np.testing.assert_allclose(self.e2.shape, (self.num_f, 3))
    def test_number_e3_edges(self):
        np.testing.assert_allclose(self.e3.shape, (self.num_f, 3))
    
    def test_number_vertices(self):
        np.testing.assert_allclose(self.v.shape, (self.num_v, 3))
    
    def test_Fa(self):
        np.testing.assert_allclose(self.Fa, self.f[:, 0])
    def test_Fb(self):
        np.testing.assert_allclose(self.Fb, self.f[:, 1])
    def test_Fc(self):
        np.testing.assert_allclose(self.Fc, self.f[:, 2])
    def test_V1_shape(self):
        np.testing.assert_allclose(self.V1.shape, (self.num_f, 3))
    def test_V2_shape(self):
        np.testing.assert_allclose(self.V1.shape, (self.num_f, 3))
    def test_V3_shape(self):
        np.testing.assert_allclose(self.V3.shape, (self.num_f, 3))
    def test_e1_vertex_map_shape(self):
        np.testing.assert_allclose(self.e1_vertex_map.shape, (self.num_f, 2))
    def test_e2_vertex_map_shape(self):
        np.testing.assert_allclose(self.e2_vertex_map.shape, (self.num_f, 2))
    def test_e3_vertex_map_shape(self):
        np.testing.assert_allclose(self.e3_vertex_map.shape, (self.num_f, 2))

    def test_e1_vertex_map_values(self):
        np.testing.assert_allclose(self.e1_vertex_map, np.stack((self.f[:, 1], self.f[:, 0]), axis=1))
    def test_e2_vertex_map_values(self):
        np.testing.assert_allclose(self.e2_vertex_map, np.stack((self.f[:, 2], self.f[:, 1]), axis=1))
    def test_e3_vertex_map_values(self):
        np.testing.assert_allclose(self.e3_vertex_map, np.stack((self.f[:, 0], self.f[:, 2]), axis=1))

    def test_normal_face_shape(self):
        np.testing.assert_allclose(self.normal_face.shape, (self.num_f, 3))
    
    def test_e1_normal_shape(self):
        np.testing.assert_allclose(self.e1_normal.shape, (self.num_f, 3))
    def test_e2_normal_shape(self):
        np.testing.assert_allclose(self.e2_normal.shape, (self.num_f, 3))
    def test_e3_normal_shape(self):
        np.testing.assert_allclose(self.e3_normal.shape, (self.num_f, 3))
    def test_center_face(self):
        np.testing.assert_allclose(self.center_face.shape, (self.num_f, 3))

    def test_e1_edge_searching_shape(self):
        np.testing.assert_allclose(self.e1_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind3b.shape, (self.num_f,))
    def test_e2_edge_searching_shape(self):
        np.testing.assert_allclose(self.e2_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind3b.shape, (self.num_f,))
    def test_e3_edge_searching_shape(self):
        np.testing.assert_allclose(self.e3_ind1b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind2b.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind3b.shape, (self.num_f,))
    def test_e1_vertex_map_searching(self):
        np.testing.assert_allclose(self.e1_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e1_ind3b_new.shape, (self.num_f,))
    def test_e2_vertex_map_searching(self):
        np.testing.assert_allclose(self.e2_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e2_ind3b_new.shape, (self.num_f,))
    def test_e3_vertex_map_searching(self):
        np.testing.assert_allclose(self.e3_ind1b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind2b_new.shape, (self.num_f,))
        np.testing.assert_allclose(self.e3_ind3b_new.shape, (self.num_f,))

    def test_e1_searching_matches(self):
        np.testing.assert_allclose(self.e1_ind1b_new, self.e1_ind1b)
        np.testing.assert_allclose(self.e1_ind2b_new, self.e1_ind2b)
        np.testing.assert_allclose(self.e1_ind3b_new, self.e1_ind3b)
    def test_e2_searching_matches(self):
        np.testing.assert_allclose(self.e2_ind1b_new, self.e2_ind1b)
        np.testing.assert_allclose(self.e2_ind2b_new, self.e2_ind2b)
        np.testing.assert_allclose(self.e2_ind3b_new, self.e2_ind3b)
    def test_e3_searching_matches(self):
        np.testing.assert_allclose(self.e3_ind1b_new, self.e3_ind1b)
        np.testing.assert_allclose(self.e3_ind2b_new, self.e3_ind2b)
        np.testing.assert_allclose(self.e3_ind3b_new, self.e3_ind3b)

    def test_normal_face_function(self):
        normal_face = wavefront.normal_face(self.v, self.f)
        np.testing.assert_allclose(normal_face, self.normal_face)
    def test_center_face_function(self):
        cof = wavefront.center_of_face(self.v, self.f)
        np.testing.assert_allclose(cof, self.center_face)

class TestDistArray():
    pt = np.array([5, 0, 0])
    array = np.array([[0, 0, 0],
                      [1, 1, 1],
                      [1, 2, 3]])
    array_random = np.random.rand(100, 3)

    def test_known_array(self):
        dist, ind = wavefront.dist_array(self.pt, self.array)
        np.testing.assert_allclose(ind, 1)
        np.testing.assert_allclose(dist, np.linalg.norm([4, -1, -1]))

class TestSignOfLargest():

    def test_positive_numbers(self):

        array = np.arange(1, 5)
        sgn = wavefront.sign_of_largest(array)
        np.testing.assert_allclose(sgn, 1)

    def test_with_zero(self):
        array = np.arange(-1, 3)
        sgn = wavefront.sign_of_largest(array)
        np.testing.assert_allclose(sgn, 1)

    def test_negative_numbers(self):
        array = np.arange(-10, 5)
        sgn = wavefront.sign_of_largest(array)
        np.testing.assert_allclose(sgn, -1)

class TestCenterOfFace():

    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    cof = wavefront.center_of_face(v_cube, f_cube)

    def test_size(self):
        np.testing.assert_allclose(self.cof.shape, self.f_cube.shape)

class TestDistanceToVerticesCubeOutsideFixedSingle():

    pt_out = np.array([1,1, 1])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    normal_face_cube = wavefront.normal_face(v_cube, f_cube)
    vertex_face_map = wavefront.vertex_face_map(v_cube, f_cube)
    D, P, F, V = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                normal_face_cube, 
                                                vertex_face_map)
    D_exp = 0.5 * np.sqrt(3)
    P_exp = np.array([0.5, 0.5, 0.5])
    F_exp = [4, 5, 6, 7, 10, 11]
    V_exp = 7

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)
    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)
    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)
    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

class TestDistanceToVerticesCubeOutsideFixedMultiple():
    """This point is equally close to an entire face of the cube

    So there are 4 vertices equidistant from pt
    """
    pt_out = np.array([1,0, 0])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    normal_face_cube = wavefront.normal_face(v_cube, f_cube)
    vertex_face_map = wavefront.vertex_face_map(v_cube, f_cube)
    D, P, F, V = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                normal_face_cube, 
                                                vertex_face_map)
    D_exp = np.ones_like(D) * 0.5 * np.sqrt(3)
    P_exp = np.array([[ 0.5, -0.5, -0.5],         
                      [ 0.5, -0.5,  0.5],         
                      [ 0.5,  0.5, -0.5],         
                      [ 0.5,  0.5,  0.5]])  
    F_exp = np.array([list([0, 6, 7, 8]),
                      list([7, 8, 9, 10]),
                      list([0, 1, 4, 6]),
                      list([4, 5, 6, 7, 10, 11])])
    V_exp = np.array([4, 5, 6, 7])

    def test_distance(self):
        np.testing.assert_allclose(np.absolute(self.D), self.D_exp)
    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)
    def test_face(self):
        for F, F_exp in zip(self.F, self.F_exp):
            np.testing.assert_allclose(F, F_exp)
    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

class TestDistanceToVerticesCubeInsideFixedSingle():

    pt_out = np.array([0.4, 0.4, 0.4])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    normal_face_cube = wavefront.normal_face(v_cube, f_cube)
    vertex_face_map = wavefront.vertex_face_map(v_cube, f_cube)
    D, P, F, V = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                normal_face_cube, 
                                                vertex_face_map)
    D_exp = 0.4 *np.sqrt(3) -0.5 * np.sqrt(3)
    P_exp = np.array([0.5, 0.5, 0.5])
    F_exp = [4, 5, 6, 7, 10, 11]
    V_exp = 7

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)
    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)
    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)
    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

class TestDistanceToVerticesCubeInsideMultiple():
    
    pt_out = np.array([0, 0, 0])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    normal_face_cube = wavefront.normal_face(v_cube, f_cube)
    vertex_face_map = wavefront.vertex_face_map(v_cube, f_cube)
    D, P, F, V = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                normal_face_cube, 
                                                vertex_face_map)
    D_exp = -np.ones_like(D) * 0.5 * np.sqrt(3)
    P_exp = v_cube
    F_exp = np.array([list([0, 1, 2, 3, 8, 9]),
                      list([3,9,10,11]),
                      list([1, 2, 4, 5]),
                      list([2, 3, 5, 11]),
                      list([0, 6, 7, 8]),
                      list([7, 8, 9, 10]),
                      list([0, 1, 4, 6]),
                      list([4, 5, 6, 7, 10, 11])])
    V_exp = np.arange(0, 8)

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)
    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)
    def test_face(self):
        for F, F_exp in zip(self.F, self.F_exp):
            np.testing.assert_allclose(F, F_exp)
    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

class TestDistanceToVerticesCubeSurfaceSingle():
    
    pt_out = np.array([0.5, 0.4, 0.4])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    normal_face_cube = wavefront.normal_face(v_cube, f_cube)
    vertex_face_map = wavefront.vertex_face_map(v_cube, f_cube)
    D, P, F, V = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                normal_face_cube, 
                                                vertex_face_map)
    D_exp = -0.1* np.sqrt(2)
    P_exp = np.array([0.5, 0.5, 0.5])
    F_exp = [4, 5, 6, 7, 10, 11]
    V_exp = 7

    def test_distance(self):
        np.testing.assert_allclose(self.D, self.D_exp)
    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)
    def test_face(self):
        np.testing.assert_allclose(self.F, self.F_exp)
    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

class TestDistanceToVerticesCubeOutsideSurfaceMultiple():
    """This point is actually on the surface of the cube

    So there are 4 vertices equidistant from pt
    """
    pt_out = np.array([0.5,0, 0])
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    normal_face_cube = wavefront.normal_face(v_cube, f_cube)
    vertex_face_map = wavefront.vertex_face_map(v_cube, f_cube)
    D, P, F, V = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                normal_face_cube, 
                                                vertex_face_map)
    D_exp = np.ones_like(D) * 0.5 * np.sqrt(2)
    P_exp = np.array([[ 0.5, -0.5, -0.5],         
                      [ 0.5, -0.5,  0.5],         
                      [ 0.5,  0.5, -0.5],         
                      [ 0.5,  0.5,  0.5]])  
    F_exp = np.array([list([0, 6, 7, 8]),
                      list([7, 8, 9, 10]),
                      list([0, 1, 4, 6]),
                      list([4, 5, 6, 7, 10, 11])])
    V_exp = np.array([4, 5, 6, 7])

    def test_distance(self):
        np.testing.assert_allclose(np.absolute(self.D), self.D_exp)
    def test_point(self):
        np.testing.assert_allclose(self.P, self.P_exp)
    def test_face(self):
        for F, F_exp in zip(self.F, self.F_exp):
            np.testing.assert_allclose(F, F_exp)
    def test_vertex(self):
        np.testing.assert_allclose(self.V, self.V_exp)

class TestDistanceToVerticesCubeInsideRandom():
    """Ranom location that is always outside the cube
    """
    pt_out = np.random.uniform(0.01 , 0.49 * np.sqrt(3)) * sphere.rand(2)
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    normal_face_cube = wavefront.normal_face(v_cube, f_cube)
    vertex_face_map = wavefront.vertex_face_map(v_cube, f_cube)
    D, P, F, V = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                normal_face_cube, 
                                                vertex_face_map)
    def test_distance(self):
        np.testing.assert_allclose(np.isfinite(self.D), True)
    def test_point(self):
        np.testing.assert_allclose(len(self.P) >= 3, True)
    def test_face(self):
        np.testing.assert_allclose(len(self.F) >= 1, True)
    def test_vertex(self):
        np.testing.assert_allclose(np.isfinite(self.V), True)

class TestDistanceToVerticesCubeOutsideRandom():
    """Ranom location that is always outside the cube
    """
    pt_out = np.random.uniform(0.51 * np.sqrt(3), 1) * sphere.rand(2)
    v_cube, f_cube = wavefront.read_obj('./integration/cube.obj')
    normal_face_cube = wavefront.normal_face(v_cube, f_cube)
    vertex_face_map = wavefront.vertex_face_map(v_cube, f_cube)
    D, P, F, V = wavefront.distance_to_vertices(pt_out, v_cube, f_cube,
                                                normal_face_cube, 
                                                vertex_face_map)
    def test_distance(self):
        np.testing.assert_allclose(np.isfinite(self.D), True)
    def test_point(self):
        np.testing.assert_allclose(len(self.P) >= 3, True)
    def test_face(self):
        np.testing.assert_allclose(len(self.F) >= 1, True)
    def test_vertex(self):
        np.testing.assert_allclose(np.isfinite(self.V), True)


