"""Try and estimate motion given true trajectory and a sequence of images

This will use Optical Flow and Feature matching from OpenCV
"""

import numpy as np
import h5py
import cv2
import matplotlib.pyplot as plt

from visual_odometry import PinholeCamera, VisualOdometry
from dynamics import asteroid, dumbbell
from visualization import plotting, blender_camera, blender
import pdb

# load imagery and state/time during those pictures
with h5py.File('./data/itokawa_landing/cycles_high_3600.hdf5', 'r') as sim_data:

    # _, camera, _, _, _, _ = blender.blender_init() 
    # K = blender_camera.get_calibration_matrix_K_from_blender(camera)
    # K = np.array(K)

    K = sim_data['K']
    i_state = sim_data['i_state']
    time = sim_data['time']
    images = sim_data['landing']
    RT_vector = sim_data['RT']
    R_i2bcam_vector = sim_data['R_i2bcam']

    num_images = images.shape[3]
    max_images = 1000
    step_size = (len(time)-1) // num_images
    # define pinhole camera model
    # pixel height of NEAR MSI camera
    width = 537
    height = 244 
    cam = PinholeCamera(width, height, K[0, 0], K[1, 1], K[0, 2], K[1, 2]) 
    vo = VisualOdometry(cam, time, i_state, RT_vector, R_i2bcam_vector)
    
    # define asteroid and dumbbell like in the simulation
    ast = asteroid.Asteroid('itokawa', 64)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
    # initialize an estimated trajectory vector
    est_time = np.zeros(max_images)
    est_pos = np.zeros((max_images, 3))
    est_R = np.zeros((max_images, 9))

    # do some math based on how many images there are and skipping frames


    # loop over the images
    for ii in range(max_images):
        time_index = (ii + 1) * step_size
        img = images[:, :, :, ii]
        
        # convert image to  gray scale  
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        vo.update(img, time_index)

        # save the estimates
        est_time[ii] = time[time_index]
        est_pos[ii, :] = vo.cur_t
        est_R[ii, :] = vo.cur_R.reshape(-1)
        
    plotting.plot_controlled_blender_inertial(time, i_state, ast, dum, fwidth=1)

# plot the estimated position vector
fig, axarr = plt.subplots(3)
axarr[0].plot(est_time, est_pos[:, 0])
axarr[1].plot(est_time, est_pos[:, 1])
axarr[2].plot(est_time, est_pos[:, 2])

plt.show(block=False)
