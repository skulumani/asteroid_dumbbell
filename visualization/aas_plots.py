import cv2
from visualization import opencv
import matplotlib.pyplot as plt
import h5py

def sift_flann_matching_image(img1, img2, ratio, plot=False, filename='/tmp/test.png'): 
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

        plt.imshow(img3)
        plt.axis('off')
        plt.show(block=False)

        plt.imsave(filename, img3, format='png')

    return matches
# load the h5py file with all the imagery and simulation data
with h5py.File('./data/itokawa_landing/cycles_high_3600.hdf5', 'r') as sim_data:
    K = sim_data['K']
    i_state = sim_data['i_state']
    time = sim_data['time']
    images = sim_data['landing']
    RT_vector = sim_data['RT']
    R_i2bcam_vector = sim_data['R_i2bcam']


    # draw some of the features from an example image
    sift_flann_matching_image(images[:, :, :, 500], images[:, :, :, 550], ratio=0.1, plot=True, filename='/tmp/itokawa_feature_matching.png')    
