#include "lidar.hpp"

#include <gtest/gtest.h>

#include <iostream>

TEST(TestLidar, ObjectCreation) {
	Lidar sensor;
	ASSERT_TRUE(sensor.view_axis.isApprox((Eigen::Vector3d() << 1, 0, 0).finished()));
}
