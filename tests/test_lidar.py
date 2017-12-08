"""Test the lidar module

"""

import numpy as np
from point_cloud import raycaster
from kinematics import attitude
import pdb

def test_object_creation():
    sensor = raycaster.Lidar()
    np.testing.assert_allclose(sensor.up_axis, np.array([0, 0, 1]))

def test_number_of_steps():
    sensor = raycaster.Lidar(num_step=10)
    np.testing.assert_allclose(sensor.lidar_arr.shape[0], 10**2)

def test_up_axis():
    sensor_y = raycaster.Lidar(up_axis=np.array([0, 1, 0]))
    sensor_z = raycaster.Lidar(up_axis=np.array([0, 0, 1]))
    R = attitude.rot1(np.pi/2)
    np.testing.assert_array_almost_equal(sensor_z.lidar_arr,
                                         R.dot(sensor_y.lidar_arr.T).T)

def test_view_axis():
    sensor_y = raycaster.Lidar(view_axis=np.array([0, 1, 0]))
    sensor_x = raycaster.Lidar()
    R = attitude.rot3(np.pi/2)
    np.testing.assert_array_almost_equal(sensor_y.lidar_arr,
                                         R.dot(sensor_x.lidar_arr.T).T) 

def test_rotation():
    R = attitude.rot3(np.pi/2)
    sensor_x = raycaster.Lidar()
    rot_arr = raycaster.Lidar().rotate_fov(R)
    np.testing.assert_array_almost_equal(rot_arr, R.dot(sensor_x.lidar_arr.T).T)

