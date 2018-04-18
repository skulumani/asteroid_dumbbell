#include "lidar.hpp"

#include <gtest/gtest.h>

#include <iostream>

TEST(TestLidar, ObjectCreation) {
	Lidar sensor;
	ASSERT_TRUE(sensor.get_view_axis().isApprox((Eigen::Vector3d() << 1, 0, 0).finished()));
}

TEST(TestLidar, NumberOfSteps) {
    int num_steps_true = 10;
    Lidar sensor;
    sensor.num_steps(num_steps_true);
    ASSERT_EQ(sensor.get_lidar_array().rows(), pow(num_steps_true, 2));
}

TEST(TestLidar, DefineAxes) {
    Eigen::Vector3d up_axis_true, view_axis_true;
    up_axis_true << 0, 1, 0;
    view_axis_true << 0, 0, -1;

    Lidar sensor;
    sensor.view_axis(view_axis_true).up_axis(up_axis_true);
    ASSERT_TRUE(sensor.get_view_axis().isApprox(view_axis_true));
    ASSERT_TRUE(sensor.get_up_axis().isApprox(up_axis_true));
}

TEST(TestLidar, UpAxis) {
    Lidar sensor_y, sensor_z;
    Eigen::Matrix<double, 3, 3> R(3, 3);
    R = Eigen::AngleAxis<double>(3.141592653589793 / 2.0, Eigen::Vector3d(1, 0, 0));
    sensor_y.up_axis((Eigen::Vector3d() << 0, 1, 0).finished());
    sensor_z.up_axis((Eigen::Vector3d() << 0, 0, 1).finished());
    
    ASSERT_TRUE(sensor_z.get_lidar_array().isApprox((R * sensor_y.get_lidar_array().transpose()).transpose()));
}

TEST(TestLidar, ViewAxis) {
    Lidar sensor_y, sensor_x;
    sensor_y.view_axis((Eigen::Vector3d() << 0, 1, 0).finished());
    Eigen::Matrix<double, 3, 3> R(3, 3);
    R = Eigen::AngleAxis<double>(3.141592653589793 / 2.0, Eigen::Vector3d(0, 0, 1));
    
    ASSERT_TRUE(sensor_y.get_lidar_array().isApprox((R * sensor_x.get_lidar_array().transpose()).transpose()));
}

TEST(TestLidar, Rotation) {
    Eigen::Matrix<double, 3, 3> R(3, 3);
    R = Eigen::AngleAxis<double>(3.141592653589793 / 2.0, Eigen::Vector3d(0, 0, 1));
    Lidar sensor_x;
    
    ASSERT_TRUE(sensor_x.rotate_fov(R).isApprox((R * sensor_x.get_lidar_array().transpose()).transpose()));
}

TEST(TestLidar, Targets) {
    Eigen::Vector3d pos;
    Eigen::Matrix<double, 3, 3> R;
    R = Eigen::AngleAxis<double>(0, Eigen::Vector3d(1, 0, 0));
    double dist = 5;
    
    pos << 1.5, 0, 0;
    Lidar sensor;
    Eigen::Matrix<double, Eigen::Dynamic, 3> targets = sensor.define_targets(pos, R, dist);
    
    Eigen::Matrix<double, 9, 3> targets_true(9, 3);
    targets_true <<
        6.48139997,  0.30467547, -0.30467547,
        6.49067399,  0.        , -0.3052427,                                 
        6.48139997, -0.30467547, -0.30467547,                                 
        6.49067399,  0.3052427 ,  0.,                                 
        6.5       ,  0.        ,  0.,                                 
        6.49067399, -0.3052427 ,  0.,                                 
        6.48139997,  0.30467547,  0.30467547,                                 
        6.49067399,  0.        ,  0.3052427,                                 
        6.48139997, -0.30467547,  0.30467547;

    ASSERT_TRUE(targets.isApprox(targets_true, 1e-3));
}
