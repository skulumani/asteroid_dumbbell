#include "reconstruct.hpp"

#include <gtest/gtest.h>

// The fixture for testing class Foo.
class TestReconstruct: public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TestReconstruct() {
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

    W_true.setOnes();
  }

  virtual ~TestReconstruct{
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
    Eigen::Matrix<double, 8, 1> W_true;
};

TEST_F(TestReconstruct, EigenConstructor) {
    ReconstructMesh reconstruct_mesh(Ve_true, Fe_true, W_true);
    ASSERT_TRUE(reconstruct_mesh.vertices.isApprox(Ve_true));
    ASSERT_TRUE(reconstruct_mesh.faces.isApprox(Fe_true));
    ASSERT_TRUE(reconstruct_mesh.weights.isApprox(W_true));
}

