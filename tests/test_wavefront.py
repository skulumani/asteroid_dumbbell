"""Test out the Wavefront reading and writing
"""

from point_cloud import wavefront
import numpy as np
import scipy.io
import pdb

def test_reader_input_sizes_itokawa_low():
    """Test that number of faces/verts matches Euler's constraint
    """
    verts, faces = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')
    
    edges_obj = verts.shape[0] + faces.shape[0] - 2
    np.testing.assert_allclose(edges_obj, 74500)
    np.testing.assert_allclose(faces.shape, (49152, 3))
    np.testing.assert_allclose(verts.shape, (25350, 3))

def test_reader_input_sizes_itokawa_high():
    """Test that number of faces/verts matches Euler's constraint
    """
    verts, faces = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_high.obj')
    
    edges_obj = verts.shape[0] + faces.shape[0] - 2
    np.testing.assert_allclose(edges_obj, 1182724)
    np.testing.assert_allclose(faces.shape, (786432, 3))
    np.testing.assert_allclose(verts.shape, (396294, 3))

# TODO: Compare with the matlab file with faces/vertices

def test_reader_matches():
    """Compare faces/verts to Matlab mat file?"""
    pass


# TODO: Test to convert OBJ to VTK to Numpy and make sure the numbers are correct
