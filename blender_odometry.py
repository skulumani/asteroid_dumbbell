"""Try and estimate motion given true trajectory and a sequence of images

This will use Optical Flow and Feature matching from OpenCV
"""

import numpy as np
import h5py
import cv2
import matplotlib.pyplot as plt

from visual_odometry import PinholeCamera, VisualOdometry

from visualization import plotting, blender_camera, blender

# load imagery and state/time during those pictures
sim_data = h5py.File('./data/itokawa_landing/blender_low.hdf5', 'r')

_, camera, _, _, _, _ = blender.blender_init() 
K = blender_camera.get_calibration_matrix_K_from_blender(camera)
K = np.array(K)

# K = sim_data['K']
# i_state = sim_data['i_state']
# time = sim_data['time']
images = sim_data['landing']
# RT_vector = sim_data['RT']
# R_i2bcam_vector = sim_data['R_i2bcam']

# define pinhole camera model
# pixel height of NEAR MSI camera
width = 537
height = 244 
cam = PinholeCamera(width, height, K[0, 0], K[1, 1], K[0, 2], K[1, 2]) 
