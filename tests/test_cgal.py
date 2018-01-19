"""Ensure that CGAL is installed and working properly
"""
import pytest
import numpy as np

CGAL = pytest.importorskip('CGAL')

def test_cgal_version():
    req_version = '4.11.0'
    cur_version = CGAL.__version__
    np.testing.assert_equal(cur_version, req_version)


