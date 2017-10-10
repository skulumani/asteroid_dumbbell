"""Testing out Point Cloud Library
"""
from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
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

def threed_harris():
    """
    https://github.com/strawlab/python-pcl/blob/master/examples/3dharris.py
    """
    cloud = pcl.load('./data/bunny.pcd')
