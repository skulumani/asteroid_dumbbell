"""Ensure that blender module is compiled and working properly

"""
import sys
import numpy as np
import pytest
pytestmark = pytest.importorskip('bpy')


def test_python_version():
    req_version = (3, 5, 4, 'final', 0)
    cur_version = sys.version_info
    np.testing.assert_equal(cur_version, req_version)

def test_anaconda_enviornment():
    asteroid_env = '/home/shankar/anaconda3/envs/asteroid'
    np.testing.assert_equal(sys.prefix, asteroid_env)

def test_blender_version():
    req_version = (2, 78, 6)
    cur_version = bpy.app.version
    np.testing.assert_equal(cur_version[0:1], req_version[0:1])

def test_blender_installed():
    result = bpy.ops.render.render(write_still=True)
    np.testing.assert_equal(result.pop(), 'FINISHED')
