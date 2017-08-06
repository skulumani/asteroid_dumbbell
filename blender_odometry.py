"""Try and estimate motion given true trajectory and a sequence of images

This will use Optical Flow and Feature matching from OpenCV
"""

import numpy as np
import h5py
import cv2
import matplotlib.pyplot as plt

from visual_odometry import PinholeCamera, VisualOdometry

# load imagery and state/time during those pictures
sim_data = h5py.File('./data/itokawa_landing/blender_low.hdf5', 'r')

K = sim_data['K']
i_state = sim_data['i_state']
time = sim_data['time']
images = sim_data['landing']
RT_vector = sim_data['RT']
R_i2bcam_vector = sim_data['R_i2bcam']

# define pinhole camera model

