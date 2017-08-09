import cv2
from visualization import opencv, plotting
from dynamics import asteroid, dumbbell, controller
import argparse
import numpy as np

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import pdb
import matplotlib.pyplot as plt
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

def plot_keyframe_trajectory(time, i_state, R_ast2int, R_bcam2i,
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
    # determine scale of translation between keyframe points
    kf_diff = np.diff(kf_traj, axis=0)
    kf_scale = np.sqrt(np.sum(kf_diff ** 2, axis=1))


    # find true positions at the same time as keyframes
    kf_traj_true = asteroid_pos[kf_time[0]:kf_time[-1], :]
    kf_scale_true = np.sqrt(np.sum((kf_traj_true[0, :] - kf_traj_true[-1,:])**2))
    
    scale = kf_scale_true
    Rb2i = R_bcam2i[kf_time[0], :].reshape((3,3))
    Rb2a = R_ast2int[kf_time[0], :].reshape((3, 3)).T.dot(Rb2i)
    
    # translate all the positions
    initial_pos_inertial_frame = i_state[0, 0:3]
   
    # rotate from camera frame to inertial frame
    kf_traj_est = scale * kf_traj + asteroid_pos[kf_time[0], :]
    
    # translate the first keyframe point

    # plot the keyframe trajectory in it's own relative frame
    kf_fig = plt.figure()
    kf_ax = axes3d.Axes3D(kf_fig)
    kf_ax.plot(kf_traj[:, 0], -kf_traj[:, 2], kf_traj[:, 1], '-*')
    kf_ax.set_zlim3d(-1, 1)
    kf_ax.set_xlim3d(-3, 3)
    kf_ax.set_ylim3d(-3, 3)
    
    kf_ax.plot(kf_traj_est[:, 0], -kf_traj_est[:, 2], np.zeros_like(kf_traj_est[:,0]), '-*')
    kf_traj_est_rot = Rb2a.dot(kf_traj_est.T).T
    # kf_ax.plot(kf_traj_est_rot[:, 0], kf_traj_est_rot[:, 1],np.zeros_like(kf_traj_est[:, 0]), '-b*')
    kf_ax.plot(kf_traj_true[:, 0], kf_traj_true[:, 1], kf_traj_true[:, 2], 'r')

    plt.show()


def create_plots(plot_flags):
    # load the h5py file with all the imagery and simulation data
    with h5py.File('./data/itokawa_landing/cycles_high_7200.hdf5', 'r') as sim_data:
        sim_data.visit(printname)
        K = sim_data['K']
        i_state = sim_data['i_state']
        time = sim_data['time']
        images = sim_data['landing']
        RT_vector = sim_data['RT']
        R_bcam2i_vector = sim_data['R_i2bcam'] # the name is incorrect - actually it's bcamera to inertial frame
        R_ast2int = sim_data['Rast2inertial']

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
            plotting.plot_controlled_blender_inertial(time, 
                                                      i_state, 
                                                      ast, 
                                                      dum, 
                                                      plot_flags.save_plots, 
                                                      1, 
                                                      controller.traverse_then_land_vertically,
                                                      controller.body_fixed_pointing_attitude)

        # create animation
        if plot_flags.animation:
            plotting.animate_inertial_trajectory(time, i_state, ast, dum, 3600, plot_flags.save_plots)

        if plot_flags.keyframe:
            plot_keyframe_trajectory(time, i_state, R_ast2int, R_bcam2i_vector)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--feature_matching", help="Generate feature matching example", action="store_true")
    parser.add_argument("--simulation_plots", help="Generate plots of the simulation",
                        action="store_true")
    parser.add_argument("--animation", help="Generate an animation",
                        action="store_true")
    
    parser.add_argument("--save_plots", help="Save plots to /tmp", action="store_true")
    
    parser.add_argument("--keyframe", help="Plot output from ORB-SLAM2", 
                        action="store_true")

    args = parser.parse_args()

    create_plots(args)

