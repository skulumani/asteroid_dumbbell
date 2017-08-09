import cv2
from visualization import opencv, plotting
from dynamics import asteroid, dumbbell, controller
import argparse

import pdb
import matplotlib.pyplot as plt
import h5py

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

def create_plots(plot_flags):
    # load the h5py file with all the imagery and simulation data
    with h5py.File('./data/itokawa_landing/cycles_high_7200.hdf5', 'r') as sim_data:
        K = sim_data['K']
        i_state = sim_data['i_state']
        time = sim_data['time']
        images = sim_data['landing']
        RT_vector = sim_data['RT']
        R_i2bcam_vector = sim_data['R_i2bcam']

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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--feature_matching", help="Generate feature matching example", action="store_true")
    parser.add_argument("--simulation_plots", help="Generate plots of the simulation",
                        action="store_true")
    parser.add_argument("--animation", help="Generate an animation",
                        action="store_true")
    
    parser.add_argument("--save_plots", help="Save plots to /tmp", action="store_true")

    args = parser.parse_args()

    create_plots(args)

