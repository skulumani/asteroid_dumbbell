#include "geodesic.hpp"

#include <gtest/gtest.h>

#include <iostream>

// The fixture for testing class Foo.
class TestGeodesic: public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TestGeodesic() {
    // You can do set-up work for each test here.
    // initialize the mesh
    Ve_true << -0.5,   -0.5, -0.5,
            -0.5, -0.5, 0.5,
            -0.5, 0.5,  -0.5,
            -0.5, 0.5,  0.5,
            0.5,  -0.5, -0.5,
            0.5,  -0.5, 0.5,
            0.5,  0.5,  -0.5,
            0.5,  0.5,  0.5;

    Fe_true << 1, 7, 5,
            1, 3, 7,
            1, 4, 3,
            1, 2, 4,
            3, 8, 7,
            3, 4, 8,
            5, 7, 8,
            5, 8, 6,
            1, 5, 6,
            1, 6, 2,
            2, 6, 8,
            2, 8, 4;
    Fe_true = Fe_true.array() - 1;

  }

  virtual ~TestGeodesic() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
    const std::string input_file = "./integration/cube.obj";
    Eigen::Matrix<double, 8, 3> Ve_true;
    Eigen::Matrix<int, 12, 3> Fe_true;
};

TEST_F(TestGeodesic, SphericalDistanceZero) {

    Eigen::Matrix<double, 1, 3> s1(3);
    Eigen::Matrix<double, 1, 3> s2(3);
    s1 << 1, 0, 0;
    s2 << 1, 0, 0;
    
    Eigen::VectorXd delta_sigma;
    delta_sigma = central_angle(s1, s2);
    
    ASSERT_EQ(delta_sigma(0), 0);
}

TEST_F(TestGeodesic, SphericalDistanceNinety) {
    Eigen::Matrix<double, 1, 3> s1(3);
    Eigen::Matrix<double, 1, 3> s2(3);
    s1 << 1, 0, 0;
    s2 << 0, 0, 1;
    
    Eigen::VectorXd delta_sigma;
    delta_sigma = central_angle(s1, s2);
    
    ASSERT_NEAR(delta_sigma(0), 1.5707963267, 1e-5 );
    
}

TEST_F(TestGeodesic, SphericalDistanceZeroArray) {
    Eigen::Matrix<double, 1, 3> s1(3);
    Eigen::Matrix<double, 2, 3> s2(2, 3);

    s1 << 1, 0, 0;
    s2 << 1, 0, 0, 1, 0, 0;
    
    Eigen::VectorXd delta_sigma;
    delta_sigma = central_angle(s1, s2);
    
    ASSERT_EQ(delta_sigma(0), 0);
    ASSERT_EQ(delta_sigma(1), 0);

}

TEST_F(TestGeodesic, Cartesian2Spherical) {
    Eigen::Matrix<double, 1, 3> c1(3);
    Eigen::Matrix<double, 1, 3> s1_true(3), s1(3);

    c1 << 1, 0, 0;
    s1_true << 1, 0, 0;
    
    s1 = cartesian2spherical(c1);

    ASSERT_TRUE(s1.isApprox(s1_true));
}

TEST_F(TestGeodesic, Cartesian2SphericalArray) {
    Eigen::Matrix<double, 2, 3> c1(2, 3);
    Eigen::Matrix<double, 2, 3> s1_true(2, 3), s1(2, 3);

    c1 << 1, 0, 0, 1, 0, 0;
    s1_true << 1, 0, 0, 1, 0, 0;
    
    s1 = cartesian2spherical(c1);

    ASSERT_TRUE(s1.isApprox(s1_true));
}

TEST_F(TestGeodesic, Spherical2Cartesian) {
    Eigen::Matrix<double, 1, 3> c1(3), c1_true(3);
    Eigen::Matrix<double, 1, 3> s1(3);
    
    s1 << 1, 0, 0;
    c1_true << 1, 0, 0;
    
    c1 = spherical2cartesian(s1);

    ASSERT_TRUE(c1.isApprox(c1_true));
}

TEST_F(TestGeodesic, Spherical2CartesianArray) {
    Eigen::Matrix<double, 2, 3> c1(2, 3), c1_true(2, 3);
    Eigen::Matrix<double, 2, 3> s1(2, 3);

    c1_true << 1, 0, 0, 1, 0, 0;
    s1 << 1, 0, 0, 1, 0, 0;
    
    c1 = spherical2cartesian(s1);

    ASSERT_TRUE(c1.isApprox(c1_true));
}

TEST(TestAzimuth, WikipediaExample) {
    Eigen::Matrix<double, 1, 3> initial_point(3), final_point(3);
    initial_point << 6378.137, deg2rad(-33), deg2rad(-71.6);
    final_point << 6378.137, deg2rad(31.4), deg2rad(121.8);

    Eigen::Matrix<double, 1, 2> azimuth = course_azimuth(initial_point, final_point);
    
    ASSERT_NEAR(azimuth(0), deg2rad(-94.413131), 1e-4);
    ASSERT_NEAR(azimuth(1), deg2rad(-78.42), 1e-2);
}

TEST(TestDegree2Radians, SomeCommonValues) {
    ASSERT_EQ(deg2rad(0), 0);
    ASSERT_EQ(deg2rad(90), kPI / 2);
    ASSERT_EQ(deg2rad(180), kPI);
    ASSERT_EQ(deg2rad(-180), -kPI);
}

TEST(TestRadians2Degree, SomeCommonValues) {
    ASSERT_EQ(rad2deg(0), 0);
    ASSERT_EQ(rad2deg(kPI), 180);
    ASSERT_EQ(rad2deg(kPI / 2), 90);
    ASSERT_EQ(rad2deg(-kPI / 2), -90);
}

TEST(TestDegree2Radians, EigenArrayCommonValues) {
    Eigen::Matrix<double, 5, 1> degrees(5), radians(5), radians_true(5);
    degrees << 0, 45, -90, 180, 360;
    radians_true << 0, kPI / 4, -kPI/2, kPI, 2 * kPI;
    radians = deg2rad(degrees);
    ASSERT_TRUE(radians.isApprox(radians_true));
}

TEST(TestRadians2Degree, EigenArrayCommonValues) {
    Eigen::Matrix<double, 5, 1> degrees(5), radians(5), degrees_true(5);
    degrees_true << 0, 45, -90, 180, 360;
    radians << 0, kPI / 4, -kPI/2, kPI, 2 * kPI;
    degrees = rad2deg(radians);
    ASSERT_TRUE(degrees.isApprox(degrees_true));
}

TEST(TestWaypoint, EquatorialPlain) {
    Eigen::Matrix<double, 1, 3> initial_point(3), final_point(3);
    initial_point << 1, 0, 0;
    final_point << 0, 1, 0;
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints = sphere_waypoint(initial_point, final_point, 5);

    ASSERT_TRUE(waypoints.row(0).isApprox(initial_point));
    ASSERT_TRUE(waypoints.row(4).isApprox(final_point, 1e-6));
}

TEST(TestWaypoint, ZeroDegreeVector) {
    Eigen::Matrix<double, 1, 3> initial_point(3), final_point(3);
    initial_point << 1, 1, 1;
    final_point << 1, 1, 1;
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints = sphere_waypoint(initial_point, final_point, 5);
    
    ASSERT_TRUE(waypoints.row(0).isApprox(initial_point));
    ASSERT_TRUE(waypoints.row(1).isApprox((Eigen::RowVector3d() << 0 , 0 ,0).finished()));
}

TEST(TestWaypoint, PiDegreeVector) {
    Eigen::Matrix<double, 1, 3> initial_point(3), final_point(3);
    initial_point << 1, 1, 1;
    final_point << -0.5, -0.5, -0.5;

    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints = sphere_waypoint(initial_point, final_point, 5);
    
    ASSERT_TRUE(waypoints.row(0).isApprox(initial_point));
    ASSERT_TRUE(waypoints.row(4).isApprox((Eigen::RowVector3d() << -1, -1, -1).finished(), 1e-6));
}

TEST(TestWaypoint, HalfPiDegreeVector) {
    Eigen::Matrix<double, 1, 3> initial_point(3), final_point(3);
    initial_point << 1, 1, 1;
    final_point << -0.5, 0.5,  0.5;

    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints = sphere_waypoint(initial_point, final_point, 5);

    ASSERT_TRUE(waypoints.row(0).isApprox(initial_point));
    ASSERT_TRUE(waypoints.row(4).isApprox((Eigen::RowVector3d() << -1, 1, 1).finished(), 1e-6));
}


