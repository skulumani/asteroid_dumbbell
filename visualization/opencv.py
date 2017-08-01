"""Module to test image processing of simulated imagery

OpenCV 3.2.0 Examples

Follows the documentation 

.. [1] http://docs.opencv.org/3.2.0/d6/d00/tutorial_py_root.html
"""

import cv2
import numpy as np
import sys
import pdb

import matplotlib.pyplot as plt
import matplotlib.image as mpimage

def harris_corner_detector(filename, plot=False):
    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray, 2, 3, 0.04)

    # result is dilated for marking the corners
    dst = cv2.dilate(dst, None)

    # threshold for an optimal value
    corner_test = dst > 0.01 * dst.max()
    corners = np.asarray(np.where(corner_test)).T
    
    if plot:
        img[corner_test] = [0, 0, 255]
        imgplot = plt.imshow(img)
        plt.show()
    
    return np.vstack((corners[:,1], corners[:,0])).T

def refined_harris_corner_detector(filename, plot=False):
    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # find the Harris corners
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray, 2, 3, 0.04)
    dst = cv2.dilate(dst, None)
    ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find the centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)

    # now draw everything to a plot
    res = np.hstack((centroids, corners))
    res = np.int0(res)
    img[res[:, 1], res[:, 0]] = [0, 0, 255]
    img[res[:, 3], res[:, 2]] = [0, 255, 0]

    if plot:
        plt.imshow(img)
        plt.show()

    return corners

def shi_tomasi_corner_detector(filename, num_features=25, plot=False):
    """
    Output :
        corners - pixel locations of 'good' features to track
    """

    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners = cv2.goodFeaturesToTrack(gray, num_features, 0.01, 10)
    corners = np.int0(corners)

    if plot:
        for i in corners:
            x, y = i.ravel()
            cv2.circle(img, (x,y), 3, 255, -1)

        plt.imshow(img)
        plt.show()

    return np.squeeze(corners)

def orb(filename, plot=False):
    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    orb = cv2.ORB_create()
    kp, des = orb.detectAndCompute(gray, None)
   
    if plot:
        img2 = img
        cv2.drawKeypoints(img, kp, img2, color=(0, 255, 0), flags=0)
        plt.figure()
        plt.imshow(img2)
        plt.show()

    return kp, des

def compare_feature_detection(filename):
    corners_harris = harris_corner_detector(filename)
    corners_harris_refined = refined_harris_corner_detector(filename)
    corners_shi = shi_tomasi_corner_detector(filename)
    kp, des = orb(filename)

    img = cv2.imread(filename)
    cv2.drawKeypoints(img, kp, img, color=(255,255,0), flags=0)

    fig, ax = plt.subplots()
    ax.imshow(img)

    ax.plot(corners_harris[:,0], corners_harris[:, 1], 'rx', label='Harris Corners')
    ax.plot(corners_harris_refined[:,0],corners_harris_refined[:, 1], 'gx', label='Harris Refined')
    ax.plot(corners_shi[:,0], corners_shi[:,1], 'bx', label='Shi-Tomasi')
    plt.legend()
    plt.show()

