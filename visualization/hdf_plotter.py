"""Generate plots from a h5py simulation output
"""

import cv2
from visualization import opencv, plotting
from dynamics import asteroid, dumbbell, controller
from kinematics import attitude
import argparse
import numpy as np

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import pdb
import h5py


def printname(name):
    print(name)

def sift_flann_matching_image(img1, img2, ratio, plot=False, 
                              filename='/tmp/test.png', save_fig=False): 
    """Need full color images
    """
    kp1, des1, _ = opencv.sift_image(img1)
    kp2, des2, _ = opencv.sift_image(img2)

    # FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)  # or empty dictionary

    flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    matches = flann.knnMatch(des1, des2, k=2)

    # draw only good matches by creating a mask
    matchesMask = [[0, 0] for i in range(len(matches))]

    # ratio test 
    for i, (m, n) in enumerate(matches):
        if m.distance < ratio * n.distance:
            matchesMask[i] = [1, 0]

    if plot:
        draw_params = dict(matchColor = (0, 255, 0),
                        singlePointColor = (255, 0, 0),
                        matchesMask = matchesMask,
                        flags = 0)

        img3 = cv2.drawMatchesKnn(img1, kp1, img2, kp2, matches, None, **draw_params)
        
        fig, ax = plt.subplots(1)
        ax.imshow(img3)
        ax.axis('off')
        if save_fig:
            plt.imsave(filename, img3, format='png')

        plt.show()
    return matches

def plot_keyframe_original(time, i_state, R_ast2int, R_bcam2i, save_fig=False,
                             fwidth=1, filename='/tmp/estimate.eps',
                             kf_path='./data/itokawa_landing/cycles_high_7200_keyframe_poses_remove_first_kf.txt'):
    """Plot keyframe trajectory without any transformation
    """
    # convert inertial position into asteriod fixed frame
    inertial_pos = i_state[:, 0:3]
    asteroid_pos = np.zeros_like(inertial_pos)

    for ii, (ip, Ra2i) in enumerate(zip(inertial_pos, R_ast2int)):
        asteroid_pos[ii, :] = Ra2i.reshape((3,3)).T.dot(ip)

    # first determine the scale of the keyframe translations
    kf_data = np.loadtxt(kf_path)
    kf_time = kf_data[:, 0].astype(dtype='int') # time of keyframe, matches image/time vector
    kf_traj = kf_data[:, 1:4] # postiion of each frame relative to the first
    kf_quat = kf_data[:, 4:8] # rotation from first keyframe to current
    
    Rcam2ast = attitude.rot3(np.deg2rad(-90)).dot(np.array([[1, 0, 0],
                         [0, 0, 1],
                         [0, 1, 0]]))

    kf_traj = Rcam2ast.dot(kf_traj.T).T
    
    # need R at time of first frame and then the asteroid to inertial - dumbbell to ast needed
    R0_s2i = i_state[653, 6:15].reshape((3, 3))
    R0_a2i = R_ast2int[653, :].reshape((3, 3))

    R0_s2a = (R0_a2i.T.dot(R0_s2i))
    
    kf_traj = R0_s2a.dot(kf_traj.T).T

    kf_R_first2cur = np.zeros((len(kf_time), 9))
    # transform each quaternion to a rotation matrix
    for ii,q in enumerate(kf_quat):
        kf_R_first2cur[ii, :] = R0_s2a.dot(Rcam2ast.dot(attitude.quattodcm(q))).reshape(-1)

    # rotate each keyframe point by the corresponding angle of asteroid
    # for ii,index in enumerate(kf_time):
        # kf_traj[ii,:] = R_ast2int[ii,:].reshape((3,3)).T.dot(kf_traj[ii,:])
        # kf_R_first2cur[ii, :] = R_ast2int[ii, :].reshape((3, 3)).T.dot(kf_R_first2cur[ii, :].reshape((3,3))).reshape(-1)

    # determine scale of translation between keyframe points
    kf_diff = np.diff(kf_traj, axis=0)
    kf_scale = np.sqrt(np.sum(kf_diff ** 2, axis=1))
    
    # find true positions at the same time as keyframes
    kf_traj_true = asteroid_pos[kf_time[0]:kf_time[-1], :]
    kf_scale_true = np.sqrt(np.sum(kf_traj_true ** 2, axis=1))
    scale = kf_scale_true[0]
    Rb2i = R_bcam2i[kf_time[0], :].reshape((3,3))
    Rb2a = R_ast2int[kf_time[0], :].reshape((3, 3)).T.dot(Rb2i)
    
    # scale and translate
    kf_traj = scale * kf_traj + asteroid_pos[kf_time[0], :]
    
    # translate kf_traj
    difference = kf_traj[0, :] - kf_traj_true[0, :]
    kf_traj = kf_traj - difference
    # plot keyframe motion without any modifications
    kf_orig_fig = plt.figure()
    kf_orig_ax = axes3d.Axes3D(kf_orig_fig)
    
    kf_orig_ax.set_zlim3d(-1, 1)
    kf_orig_ax.set_xlim3d(-0, 2)
    kf_orig_ax.set_ylim3d(-1, 1)
    kf_orig_ax.plot(kf_traj[:,0], kf_traj[:, 1], kf_traj[:, 2], 'b-*')


    # plot the viewing direction
    length = 0.3
    for ii, R in enumerate(kf_R_first2cur):
        view_axis = R.reshape((3,3))[:, 2]
        kf_orig_ax.plot([kf_traj[ii, 0], kf_traj[ii, 0] + length * view_axis[0]], 
                        [kf_traj[ii, 1], kf_traj[ii, 1] + length * view_axis[1]],
                        [kf_traj[ii, 2], kf_traj[ii, 2] + length * view_axis[2]],
                        'r')

    kf_orig_ax.plot(kf_traj_true[:, 0], kf_traj_true[:, 1], kf_traj_true[:, 2], 'r')
    kf_orig_ax.set_title('Keyframe Original')
    kf_orig_ax.set_xlabel('X')
    kf_orig_ax.set_ylabel('Y')
    kf_orig_ax.set_zlabel('Z')

    # plot the components
    kf_comp_fig, kf_comp_ax = plt.subplots(3, 1, figsize=plotting.figsize(1), sharex=True)
    kf_comp_ax[0].plot(kf_time, kf_traj[:, 0], 'b-*', label='Estimate')
    kf_comp_ax[0].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 0], 'r-', label='True')
    kf_comp_ax[0].set_ylim(0, 3)
    kf_comp_ax[0].set_ylabel(r'$X$ (km)')
    kf_comp_ax[0].grid()

    kf_comp_ax[1].plot(kf_time, kf_traj[:, 1], 'b-*', label='Estimate')
    kf_comp_ax[1].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 1], 'r-', label='True')
    kf_comp_ax[1].set_ylim(-2, 1)
    kf_comp_ax[1].set_ylabel(r'$Y$ (km)')
    kf_comp_ax[1].grid()

    kf_comp_ax[2].plot(kf_time, kf_traj[:, 2], 'b-*', label='Estimate')
    kf_comp_ax[2].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 2], 'r-', label='True')
    kf_comp_ax[2].set_ylim(-0.5, 2.5)
    kf_comp_ax[2].set_ylabel(r'$Z$ (km)')
    kf_comp_ax[2].grid()

    kf_comp_ax[2].set_xlabel('Time (sec)')
    plt.legend(loc='best')

    if save_fig:
        plt.figure(kf_comp_fig.number)
        plt.savefig(filename)

    plt.show()

def plot_keyframe_original_asteroid(time, state, R_bcam2a, save_fig=False,
                             fwidth=1, filename='/tmp/estimate.eps',
                             kf_path='./data/asteroid_circumnavigate/asteroid_fixed_circumnavigate.txt'):
    """Plot keyframe trajectory and scale to match truth

    The asteroid is assumed to be not rotating. Therefore the input state is 
    actually already in the asteroid frame
    """
    # convert inertial position into asteriod fixed frame
    asteroid_pos = state[:, 0:3]

    # first determine the scale of the keyframe translations
    kf_data = np.loadtxt(kf_path)
    kf_time = kf_data[:, 0].astype(dtype='int') # time of keyframe, matches image/time vector
    kf_traj = kf_data[:, 1:4] # postiion of each frame relative to the first
    kf_quat = kf_data[:, 4:8] # rotation from first keyframe to current
    
    Rcam2ast = attitude.rot3(np.deg2rad(-90)).dot(np.array([[1, 0, 0],
                         [0, 0, 1],
                         [0, 1, 0]]))

    kf_traj = Rcam2ast.dot(kf_traj.T).T
    
    # need R at time of first frame and then the asteroid to inertial - dumbbell to ast needed
    R0_s2i = state[kf_time[0], 6:15].reshape((3, 3))

    R0_s2a = (R0_s2i)
    
    kf_traj = R0_s2a.dot(kf_traj.T).T

    kf_R_first2cur = np.zeros((len(kf_time), 9))
    # transform each quaternion to a rotation matrix
    for ii,q in enumerate(kf_quat):
        kf_R_first2cur[ii, :] = R0_s2a.dot(Rcam2ast.dot(attitude.quattodcm(q))).reshape(-1)

    # determine scale of translation between keyframe points
    kf_diff = np.diff(kf_traj, axis=0)
    kf_scale = np.sqrt(np.sum(kf_diff ** 2, axis=1))
    
    # find true positions at the same time as keyframes
    kf_traj_true = asteroid_pos[kf_time[0]:kf_time[-1], :]
    kf_scale_true = np.sqrt(np.sum(kf_traj_true ** 2, axis=1))
    scale = kf_scale_true[0]
    Rb2a = R_bcam2a[kf_time[0], :].reshape((3,3))
    
    # scale and translate
    kf_traj = scale * kf_traj + asteroid_pos[kf_time[0], :]
    
    # translate kf_traj
    difference = kf_traj[0, :] - kf_traj_true[0, :]
    kf_traj = kf_traj - difference
    # plot keyframe motion without any modifications
    kf_orig_fig = plt.figure()
    kf_orig_ax = axes3d.Axes3D(kf_orig_fig)
    
    kf_orig_ax.set_zlim3d(-1, 1)
    kf_orig_ax.set_xlim3d(-4, 4)
    kf_orig_ax.set_ylim3d(-4, 4)
    kf_orig_ax.plot(kf_traj[:,0], kf_traj[:, 1], kf_traj[:, 2], 'b-*')


    # plot the viewing direction
    length = 0.3
    for ii, R in enumerate(kf_R_first2cur):
        view_axis = R.reshape((3,3))[:, 2]
        kf_orig_ax.plot([kf_traj[ii, 0], kf_traj[ii, 0] + length * view_axis[0]], 
                        [kf_traj[ii, 1], kf_traj[ii, 1] + length * view_axis[1]],
                        [kf_traj[ii, 2], kf_traj[ii, 2] + length * view_axis[2]],
                        'r')

    kf_orig_ax.plot(kf_traj_true[:, 0], kf_traj_true[:, 1], kf_traj_true[:, 2], 'r')
    kf_orig_ax.set_title('Keyframe Original')
    kf_orig_ax.set_xlabel('X')
    kf_orig_ax.set_ylabel('Y')
    kf_orig_ax.set_zlabel('Z')

    # plot the components
    kf_comp_fig, kf_comp_ax = plt.subplots(3, 1, figsize=plotting.figsize(1), sharex=True)
    kf_comp_ax[0].plot(kf_time, kf_traj[:, 0], 'b-*', label='Estimate')
    kf_comp_ax[0].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 0], 'r-', label='True')
    kf_comp_ax[0].set_ylim(-4, 4)
    kf_comp_ax[0].set_ylabel(r'$X$ (km)')
    kf_comp_ax[0].grid()

    kf_comp_ax[1].plot(kf_time, kf_traj[:, 1], 'b-*', label='Estimate')
    kf_comp_ax[1].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 1], 'r-', label='True')
    kf_comp_ax[1].set_ylim(-4, 4)
    kf_comp_ax[1].set_ylabel(r'$Y$ (km)')
    kf_comp_ax[1].grid()

    kf_comp_ax[2].plot(kf_time, kf_traj[:, 2], 'b-*', label='Estimate')
    kf_comp_ax[2].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 2], 'r-', label='True')
    kf_comp_ax[2].set_ylim(-1, 1)
    kf_comp_ax[2].set_ylabel(r'$Z$ (km)')
    kf_comp_ax[2].grid()

    kf_comp_ax[2].set_xlabel('Time (sec)')
    plt.legend(loc='best')

    # compute the difference in the components
    traj_diff = np.absolute(asteroid_pos[kf_time, :] - kf_traj)
    kf_diff_fig, kf_diff_ax = plt.subplots(3, 1, figsize=plotting.figsize(1), sharex=True)
    kf_diff_ax[0].plot(kf_time, traj_diff[:, 0], 'b-*')
    kf_diff_ax[0].set_ylabel(r'$X$ (km)')
    kf_diff_ax[0].grid()
    kf_diff_ax[1].plot(kf_time, traj_diff[:, 1], 'b-*')
    kf_diff_ax[1].set_ylabel(r'$Y$ (km)')
    kf_diff_ax[1].grid()
    kf_diff_ax[2].plot(kf_time, traj_diff[:, 2], 'b-*')
    kf_diff_ax[2].set_ylabel(r'$Z$ (km)')
    kf_diff_ax[2].grid()
    
    kf_diff_ax[2].set_xlabel(r'Time (sec)')
    kf_diff_ax[0].set_title('Difference between ORBSLAM estimate and truth')
    if save_fig:
        plt.figure(kf_comp_fig.number)
        plt.savefig(filename)

    plt.show()

def plot_keyframe_original_lissajous(time, state, R_bcam2a, save_fig=False,
                             fwidth=1, filename='/tmp/estimate.eps',
                             kf_path='./data/asteroid_circumnavigate/lissajous.txt'):
    """Plot keyframe trajectory and scale to match truth

    The asteroid is assumed to be not rotating. Therefore the input state is 
    actually already in the asteroid frame
    """
    # convert inertial position into asteriod fixed frame
    asteroid_pos = state[:, 0:3]

    # first determine the scale of the keyframe translations
    kf_data = np.loadtxt(kf_path)
    kf_time = kf_data[:, 0].astype(dtype='int') # time of keyframe, matches image/time vector
    kf_traj = kf_data[:, 1:4] # postiion of each frame relative to the first
    kf_quat = kf_data[:, 4:8] # rotation from first keyframe to current
    
    Rcam2ast = attitude.rot2(np.deg2rad(0)).dot(attitude.rot3(np.deg2rad(-90)).dot(np.array([[1, 0, 0],
                         [0, 0, 1],
                         [0, 1, 0]])))

    kf_traj = Rcam2ast.dot(kf_traj.T).T
    
    # need R at time of first frame and then the asteroid to inertial - dumbbell to ast needed
    R0_s2i = state[kf_time[0], 6:15].reshape((3, 3))

    R0_s2a = (R0_s2i)
    
    kf_traj = R0_s2a.dot(kf_traj.T).T

    kf_R_first2cur = np.zeros((len(kf_time), 9))
    # transform each quaternion to a rotation matrix
    for ii,q in enumerate(kf_quat):
        kf_R_first2cur[ii, :] = R0_s2a.dot(Rcam2ast.dot(attitude.quattodcm(q))).reshape(-1)

    # determine scale of translation between keyframe points
    kf_diff = np.diff(kf_traj, axis=0)
    kf_scale = np.sqrt(np.sum(kf_diff ** 2, axis=1))
    
    # find true positions at the same time as keyframes
    kf_traj_true = asteroid_pos[kf_time[0]:kf_time[-1], :]
    kf_scale_true = np.sqrt(np.sum(kf_traj_true ** 2, axis=1))
    scale = kf_scale_true[0]
    Rb2a = R_bcam2a[kf_time[0], :].reshape((3,3))
    
    # scale and translate
    kf_traj = scale * kf_traj + asteroid_pos[kf_time[0], :]
    
    # translate kf_traj
    difference = kf_traj[0, :] - kf_traj_true[0, :]
    kf_traj = kf_traj - difference
    # plot keyframe motion without any modifications
    kf_orig_fig = plt.figure()
    kf_orig_ax = axes3d.Axes3D(kf_orig_fig)
    
    kf_orig_ax.set_zlim3d(-5, 5)
    kf_orig_ax.set_xlim3d(-5, 5)
    kf_orig_ax.set_ylim3d(-5, 5)
    kf_orig_ax.plot(kf_traj[:,0], kf_traj[:, 1], kf_traj[:, 2], 'b-*')


    # plot the viewing direction
    length = 0.3
    for ii, R in enumerate(kf_R_first2cur):
        view_axis = R.reshape((3,3))[:, 2]
        kf_orig_ax.plot([kf_traj[ii, 0], kf_traj[ii, 0] + length * view_axis[0]], 
                        [kf_traj[ii, 1], kf_traj[ii, 1] + length * view_axis[1]],
                        [kf_traj[ii, 2], kf_traj[ii, 2] + length * view_axis[2]],
                        'r')

    kf_orig_ax.plot(kf_traj_true[:, 0], kf_traj_true[:, 1], kf_traj_true[:, 2], 'r')
    kf_orig_ax.set_title('Keyframe Original')
    kf_orig_ax.set_xlabel('X')
    kf_orig_ax.set_ylabel('Y')
    kf_orig_ax.set_zlabel('Z')

    # plot the components
    kf_comp_fig, kf_comp_ax = plt.subplots(3, 1, figsize=plotting.figsize(1), sharex=True)
    kf_comp_ax[0].plot(kf_time, kf_traj[:, 0], 'b-*', label='Estimate')
    kf_comp_ax[0].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 0], 'r-', label='True')
    kf_comp_ax[0].set_ylim(-5, 5)
    kf_comp_ax[0].set_ylabel(r'$X$ (km)')
    kf_comp_ax[0].grid()

    kf_comp_ax[1].plot(kf_time, kf_traj[:, 1], 'b-*', label='Estimate')
    kf_comp_ax[1].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 1], 'r-', label='True')
    kf_comp_ax[1].set_ylim(-5, 5)
    kf_comp_ax[1].set_ylabel(r'$Y$ (km)')
    kf_comp_ax[1].grid()

    kf_comp_ax[2].plot(kf_time, kf_traj[:, 2], 'b-*', label='Estimate')
    kf_comp_ax[2].plot(time[kf_time[0]:kf_time[-1]], kf_traj_true[:, 2], 'r-', label='True')
    kf_comp_ax[2].set_ylim(-5, 5)
    kf_comp_ax[2].set_ylabel(r'$Z$ (km)')
    kf_comp_ax[2].grid()

    kf_comp_ax[2].set_xlabel('Time (sec)')
    plt.legend(loc='best')

    # compute the difference in the components
    traj_diff = np.absolute(asteroid_pos[kf_time, :] - kf_traj)
    kf_diff_fig, kf_diff_ax = plt.subplots(3, 1, figsize=plotting.figsize(1), sharex=True)
    kf_diff_ax[0].plot(kf_time, traj_diff[:, 0], 'b-*')
    kf_diff_ax[0].set_ylabel(r'$X$ (km)')
    kf_diff_ax[0].grid()
    kf_diff_ax[1].plot(kf_time, traj_diff[:, 1], 'b-*')
    kf_diff_ax[1].set_ylabel(r'$Y$ (km)')
    kf_diff_ax[1].grid()
    kf_diff_ax[2].plot(kf_time, traj_diff[:, 2], 'b-*')
    kf_diff_ax[2].set_ylabel(r'$Z$ (km)')
    kf_diff_ax[2].grid()
    
    kf_diff_ax[2].set_xlabel(r'Time (sec)')
    kf_diff_ax[0].set_title('Difference between ORBSLAM estimate and truth')
    if save_fig:
        plt.figure(kf_comp_fig.number)
        plt.savefig(filename)

    plt.show()

def plot_keyframe_trajectory(time, i_state, R_ast2int, R_bcam2i, save_fig=False,
                             fwidth=1, filename='/tmp/estimate.eps',
                             kf_path='./data/itokawa_landing/cycles_high_7200_keyframe_poses.txt'):
    """Read the keyframe data and transform it to match my stuff
    """
    
    # convert inertial position into asteriod fixed frame
    inertial_pos = i_state[:, 0:3]
    asteroid_pos = np.zeros_like(inertial_pos)

    for ii, (ip, Ra2i) in enumerate(zip(inertial_pos, R_ast2int)):
        asteroid_pos[ii, :] = Ra2i.reshape((3,3)).T.dot(ip)

    # first determine the scale of the keyframe translations
    kf_data = np.loadtxt(kf_path)
    kf_time = kf_data[:, 0].astype(dtype='int') # time of keyframe, matches image/time vector
    kf_traj = kf_data[:, 1:4] # postiion of each frame relative to the first
    kf_quat = kf_data[:, 4:8] # rotation from first keyframe to current
    
    pdb.set_trace()
    Rcam2ast = np.array([[1, 0, 0],
                         [0, 0, 1],
                         [0, 1, 0]])


    kf_traj = Rcam2ast.dot(kf_traj.T).T
    
    # need R at time of first frame and then the asteroid to inertial - dumbbell to ast needed
    R0_s2i = i_state[kf_time[0], 6:15].reshape((3, 3))
    R0_a2i = R_ast2int[kf_time[0], :].reshape((3, 3))

    R0_s2a = (R0_a2i.T.dot(R0_s2i))
    
    kf_traj = R0_s2a.dot(kf_traj.T).T

    kf_R_first2cur = np.zeros((len(kf_time), 9))
    # transform each quaternion to a rotation matrix
    # for ii,q in enumerate(kf_quat):
    #     kf_R_first2cur[ii, :] = R0_s2a.dot(Rcam2ast.dot(attitude.quattodcm(q))).reshape(-1)

    # rotate each keyframe point by the corresponding angle of asteroid
    for ii,index in enumerate(kf_time):
        # kf_traj[ii,:] = R_ast2int[ii,:].reshape((3,3)).T.dot(kf_traj[ii,:])
        kf_R_first2cur[ii, :] = R_ast2int[ii, :].reshape((3, 3)).T.dot(kf_R_first2cur[ii, :].reshape((3,3))).reshape(-1)

    # determine scale of translation between keyframe points
    kf_diff = np.diff(kf_traj, axis=0)
    kf_scale = np.sqrt(np.sum(kf_diff ** 2, axis=1))
    
    # find true positions at the same time as keyframes
    kf_traj_true = asteroid_pos[kf_time, :]
    kf_scale_true = np.sqrt(np.sum(kf_traj_true ** 2, axis=1))
    scale = kf_scale_true[0]
    Rb2i = R_bcam2i[kf_time[0], :].reshape((3,3))
    Rb2a = R_ast2int[kf_time[0], :].reshape((3, 3)).T.dot(Rb2i)
    
    # scale and translate
    kf_traj = scale * kf_traj + asteroid_pos[kf_time[0], :]
    
    # plot keyframe motion without any modifications
    kf_orig_fig = plt.figure()
    kf_orig_ax = axes3d.Axes3D(kf_orig_fig)
    
    kf_orig_ax.set_zlim3d(-1, 1)
    kf_orig_ax.set_xlim3d(-3, 3)
    kf_orig_ax.set_ylim3d(-3, 3)
    kf_orig_ax.plot(kf_traj[:,0], kf_traj[:, 1], kf_traj[:, 2], 'b-*')

    # plot the viewing direction
    length = 0.3
    for ii, R in enumerate(kf_R_first2cur):
        view_axis = R.reshape((3,3))[:, 2]
        kf_orig_ax.plot([kf_traj[ii, 0], kf_traj[ii, 0] + length * view_axis[0]], 
                        [kf_traj[ii, 1], kf_traj[ii, 1] + length * view_axis[1]],
                        [kf_traj[ii, 2], kf_traj[ii, 2] + length * view_axis[2]],
                        'r')

    kf_orig_ax.plot(kf_traj_true[:, 0], kf_traj_true[:, 1], kf_traj_true[:, 2], 'r')
    kf_orig_ax.set_title('Keyframe Original')
    kf_orig_ax.set_xlabel('X')
    kf_orig_ax.set_ylabel('Y')
    kf_orig_ax.set_zlabel('Z')

    # plot the components
    kf_comp_fig, kf_comp_ax = plt.subplots(3, 1, figsize=plotting.figsize(1), sharex=True)
    kf_comp_ax[0].plot(kf_time, kf_traj[:, 0], 'b-*', label='Estimate')
    kf_comp_ax[0].plot(kf_time, kf_traj_true[:, 0], 'r-*', label='True')
    kf_comp_ax[0].set_ylim(0, 3)
    kf_comp_ax[0].set_ylabel(r'$X$ (km)')

    kf_comp_ax[1].plot(kf_time, kf_traj[:, 1], 'b-*', label='Estimate')
    kf_comp_ax[1].plot(kf_time, kf_traj_true[:, 1], 'r-*', label='True')
    kf_comp_ax[1].set_ylim(-2, 1)
    kf_comp_ax[1].set_ylabel(r'$Y$ (km)')

    kf_comp_ax[2].plot(kf_time, kf_traj[:, 2], 'b-*', label='Estimate')
    kf_comp_ax[2].plot(kf_time, kf_traj_true[:, 2], 'r-*', label='True')
    kf_comp_ax[2].set_ylim(-0.5, 2.5)
    kf_comp_ax[2].set_ylabel(r'$Z$ (km)')

    kf_comp_ax[2].set_xlabel('Time (sec)')
    plt.legend(loc='best')

    if save_fig:
        plt.figure(kf_comp_fig.number)
        plt.savefig(filename)

    plt.show()


def create_plots(filename, plot_flags):

    # logic to change the desired controller function
    if plot_flags.mission == 'lissajous':
        desired_attitude_func = controller.body_fixed_pointing_attitude
        desired_translation_func = controller.inertial_lissajous_yz_plane
    elif plot_flags.mission == 'circumnavigate':
        desired_attitude_func = controller.body_fixed_pointing_attitude
        desired_translation_func = controller.inertial_circumnavigate

    # load the h5py file with all the imagery and simulation data
    with h5py.File(filename, 'r') as sim_data:
        sim_data.visit(printname)
        K = sim_data['K']
        i_state = sim_data['i_state']
        time = sim_data['time']
        images = sim_data['landing']
        RT_vector = sim_data['RT']
        R_bcam2i_vector = sim_data['R_i2bcam'] # the name is incorrect - actually it's bcamera to inertial frame
        # R_ast2int = sim_data['Rast2inertial']

        # define the asteroid and dumbbell objects like the simulation driver
        ast_name = 'itokawa'
        num_faces = 64
        ast = asteroid.Asteroid(ast_name,num_faces)
        dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

        # draw some of the features from an example image
        if plot_flags.feature_matching:
            sift_flann_matching_image(images[:, :, :, 3000],
                                      images[:, :, :, 3200], ratio=0.3, 
                                      plot=True, 
                                      filename='/tmp/itokawa_feature_matching.png',
                                      save_fig=plot_flags.save_plots)    


        # draw the true and estimated trajectory
        if plot_flags.simulation_plots:
            if plot_flags.frame == 'inertial':
                plotting.plot_controlled_blender_inertial(time, 
                                                        i_state, 
                                                        ast, 
                                                        dum, 
                                                        plot_flags.save_plots, 
                                                        1, 
                                                        controller.traverse_then_land_vertically,
                                                        controller.body_fixed_pointing_attitude)
            elif plot_flags.frame == 'asteroid':
                plotting.plot_controlled_blender_inertial_fixed_asteroid(time, 
                                                        i_state, 
                                                        ast, 
                                                        dum, 
                                                        plot_flags.save_plots, 
                                                        1, 
                                                        desired_translation_func,
                                                        desired_attitude_func)

        # create animation
        if plot_flags.animation:
            if plot_flags.frame == 'inertial':
                plotting.animate_inertial_trajectory(time, i_state, ast, dum, 3600, plot_flags.save_plots)
            elif plot_flags.frame == 'asteroid':
                plotting.animate_relative_trajectory(time, i_state, ast, dum, plot_flags.save_plots)

        if plot_flags.keyframe:
            # plot_keyframe_trajectory(time, i_state, R_ast2int, R_bcam2i_vector, 
            #                          plot_flags.save_plots, fwidth=1, 
            #                          filename='/tmp/keyframe_estimate.eps')
            if plot_flags.mission == 'asteroid':
                plot_keyframe_original_asteroid(time, i_state, R_bcam2i_vector,
                                    plot_flags.save_plots, fwidth=1,
                                    filename='/tmp/keyframe_estimate.eps', 
                                    kf_path=plot_flags.kf_path)
            elif plot_flags.mission == 'lissajous':
                plot_keyframe_original_lissajous(time, i_state, R_bcam2i_vector,
                                    plot_flags.save_plots, fwidth=1,
                                    filename='/tmp/keyframe_estimate.eps',
                                    kf_path=plot_flags.kf_path)
        
        if plot_flags.blender_png:  # generate blender images
            output_path = './visualization/blender'
            num_images = images.shape[3]

            for ii in range(num_images):
                cv2.imwrite(output_path + '/test' + str.zfill(str(ii), 6) + '.png', images[:, :, :, ii])
                print("Saving image {0}/{1}".format(ii, num_images))

            print("Finished extracting all the images")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate plots from hdf simulation output')
    parser.add_argument('filename', type=str, help='Filename of hdf file', action='store')
    parser.add_argument('kf_path', type=str, help='Path to keyframe trajectory', action='store')
    parser.add_argument("frame", help="Reference frame - asteroid or inertial", action="store")
    parser.add_argument("mission", help="Misison to plot - lissajous or circumnavigate", action='store')

    parser.add_argument("--feature_matching", help="Generate feature matching example", action="store_true")
    parser.add_argument("--simulation_plots", help="Generate plots of the simulation",
                        action="store_true")
    parser.add_argument("--animation", help="Generate an animation",
                        action="store_true")
    
    parser.add_argument("--save_plots", help="Save plots to /tmp", action="store_true")
    
    parser.add_argument("--keyframe", help="Plot output from ORB-SLAM2", 
                        action="store_true")
    parser.add_argument("--blender_png", help="Generate a series of Blender PNG images from the simulation",
                        action='store_true')

    args = parser.parse_args()
    create_plots(args.filename, args)

