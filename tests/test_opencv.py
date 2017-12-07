"""Ensure that blender module is compiled and working properly

"""
import sys
import numpy as np
import pytest
cv2 = pytest.importorskip('cv2')

def test_python_version():
    req_version = (3, 5, 4, 'final', 0)
    cur_version = sys.version_info
    np.testing.assert_equal(cur_version, req_version)

def test_anaconda_enviornment():
    asteroid_env = '/home/shankar/anaconda3/envs/asteroid'
    np.testing.assert_equal(sys.prefix, asteroid_env)

def test_opencv_version():
    req_version = '3.2.0'
    cur_version = cv2.__version__
    np.testing.assert_equal(cur_version, req_version)

