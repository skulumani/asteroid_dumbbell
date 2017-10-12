"""Testing out Point Cloud Library
"""
from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pdb
import pcl

def segment_cyl_plane(filename='./data/table_scene_mug_stereo_textured.pcd'):
    """Segment and create a cloud
    
    https://github.com/strawlab/python-pcl/blob/master/examples/segment_cyl_plane.py

    http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php

    """

    cloud = pcl.load(filename)

    print(cloud.size)
    
    fil = cloud.make_passthrough_filter()
    fil.set_filter_field_name("z")
    fil.set_filter_limits(0, 1.5)
    cloud_filtered = fil.filter()

    print(cloud_filtered.size)

    seg = cloud_filtered.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(100)
    seg.set_distance_threshold(0.03)
    indices, model = seg.segment()

    print(model)

    cloud_plane = cloud_filtered.extract(indices, negative=False)
    pcl.save(cloud_plane, './data/plane.pcd')
    
    cloud_cyl = cloud_filtered.extract(indices, negative=True)

    seg = cloud_cyl.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_CYLINDER)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(10000)
    seg.set_distance_threshold(0.05)
    seg.set_radius_limits(0, 0.1)
    indices, model = seg.segment()

    print(model)

    cloud_cylinder = cloud_cyl.extract(indices, negative=False)
    pcl.save(cloud_cylinder, './data/cylinder.pcd')
    
def visualization():
    """Visualization testing

    https://github.com/strawlab/python-pcl/blob/master/examples/visualization.py

    """

    try:  
        import pcl.pcl_visualization
        # from pcl.pcl_registration import icp, gicp, icp_nl

        cloud = pcl.load_XYZRGB('./examples/pcldata/tutorials/table_scene_mug_stereo_textured.pcd')

        visual = pcl.pcl_visualization.CloudViewing()

        # PointXYZ
        # visual.ShowMonochromeCloud(cloud)

        # visual.ShowGrayCloud(cloud)
        visual.ShowColorCloud(cloud)
        # visual.ShowColorACloud(cloud)

        # while True:
        #     visual.WasStopped()
        # end

        flag = True
        while flag:
            flag != visual.WasStopped()
        end
    except ImportError:
        print("Only works on Windows: https://github.com/strawlab/python-pcl/issues/127")

def thread_harris():
    """
    https://github.com/strawlab/python-pcl/blob/master/examples/3dharris.py
    """
    cloud = pcl.load('./data/bunny.pcd')
    print("cloud points: " + str(cloud.size))
    
    detector = cloud.make_HarrisKeypoint3D()
    detector.set_NonMaxSupression(True)
    detector.set_Radius(0)
    keypoints = detector.compute()

    print("keypoints detected: " + str(keypoints.size))

    keypoints3D = pcl.PointCloud()
    high = 0
    low = 0

    points = np.zeros((keypoints.size, 3), dtype=np.float32)
    # generate the data
    for i in range(0, keypoints.size):
        points[i, 0] = keypoints[i][0]
        points[i, 1] = keypoints[i][1]
        points[i, 2] = keypoints[i][2]
        intensity = keypoints[i][3]

        if intensity > high:
            print("coords: " + str(keypoints[i][0]) + ";" + str(keypoints[i][1]) + ";" + str(keypoints[i][2]))
            high = intensity

            if intensity < low:
                low = intensity

    keypoints3D.from_array(points)
    print("maximal response: " + str(high) + " min response: " + str(low))

    return cloud, keypoints3D 

def plot_point_cloud(cloud=pcl.load('./data/bunny.pcd')):

    # extract coordinates of points
    points = np.zeros((cloud.size, 3))
    for i in range(0, cloud.size):
        points[i, :] = np.array([cloud[i][0], cloud[i][1], cloud[i][2]])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], zdir='x')
    plt.show()

def point_cloud_testing(cloud=pcl.load('./data/bunny.pcd')):
    """Testing out plotting of point clouds
    """
    
    # properties of the cloud
    print('Width of point cloud {}.'.format(cloud.width))
    print('Height of point cloud {}.'.format(cloud.height))
    if cloud.height == 1:
        print('Unorganized point cloud')
    else:
        print("Organized point cloud")


    # extract coordinates of points
    points = np.zeros((cloud.size, 3))
    for i in range(0, cloud.size):
        points[i, :] = np.array([cloud[i][0], cloud[i][1], cloud[i][2]])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], zdir='x')
    plt.show()
