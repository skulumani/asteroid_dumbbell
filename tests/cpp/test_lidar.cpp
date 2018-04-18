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

