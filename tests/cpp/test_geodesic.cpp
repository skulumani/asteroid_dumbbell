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

    Eigen::Vector3d s1(3);
    Eigen::Matrix<double, 1, 3> s2(3);
    s1 << 1, 0, 0;
    s2 << 1, 0, 0;
    
    Eigen::VectorXd delta_sigma;
    delta_sigma = central_angle(s1, s2);
    
    ASSERT_EQ(delta_sigma(0), 0);
}

TEST_F(TestGeodesic, SphericalDistanceNinety) {
    Eigen::Vector3d s1(3);
    Eigen::Matrix<double, 1, 3> s2(3);
    s1 << 1, 0, 0;
    s2 << 0, 0, 1;
    
    Eigen::VectorXd delta_sigma;
    delta_sigma = central_angle(s1, s2);
    
    ASSERT_NEAR(delta_sigma(0), 1.5707963267, 1e-5 );
    
}

TEST_F(TestGeodesic, SphericalDistanceZeroArray) {
    Eigen::Vector3d s1(3);
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
