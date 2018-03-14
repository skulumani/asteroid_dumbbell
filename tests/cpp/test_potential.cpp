#include "potential.hpp"

#include "gtest/gtest.h"

#include <iostream>

class TestPotential: public ::testing::Test {
    protected:
        TestPotential() {
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

            Fe_true << 1, 7, 5, 1, 3, 7, 1, 4, 3, 1, 2, 4, 3, 8, 7, 3, 4, 8, 5,
                    7, 8, 5, 8, 6, 1, 5, 6, 1, 6, 2, 2, 6, 8, 2, 8, 4;
            Fe_true = Fe_true - 1;

            Fa = Fe_true.col(0);
            Fb = Fe_true.col(1);
            Fc = Fe_true.col(2);

            e1_vertex_map << 6, 0, 2, 0, 3, 0, 1, 0, 7, 2, 3, 2, 6, 4, 7, 4, 4,
                          0, 5, 0, 5, 1, 7, 1;
            e2_vertex_map << 4, 6, 6, 2, 2, 3, 3, 1, 6, 7, 7, 3, 7, 6, 5, 7, 5,
                          4, 1, 5, 7, 5, 3, 7;
            e3_vertex_map << 0, 4, 0, 6, 0, 2, 0, 3, 2, 6, 2, 7, 4, 7, 4, 5, 0,
                          5, 0, 1, 1, 7, 1, 3;

            w_face_true << 0.03808065, 0.02361574, 0.07694205, 0.07694205,
                        0.03808065, 0.02361574, -0.20033484, -0.20033484,
                        0.03808065, 0.02361574, 0.03808065,0.02361574;
        }

  virtual ~TestPotential() {
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
    Eigen::Array<double, 8, 3> Ve_true;
    Eigen::Array<int, 12, 3> Fe_true;
    Eigen::Array<int, 12, 1> Fa, Fb, Fc;
    Eigen::Array<double, 12, 3> e1, e2, e3;
    Eigen::Array<int, 12, 2> e1_vertex_map, e2_vertex_map, e3_vertex_map;
    Eigen::Array<double, 12, 1> w_face_true;

};

TEST_F(TestPotential, LaplacianFactor) {
    Eigen::Array<double, 1, 3> state;
    state << 2, 0, 0;
    Eigen::Array<double, Eigen::Dynamic, 3> r_v;
    r_v = Ve_true.rowwise() - state;
    Eigen::Array<double, 12, 1> w_face;
    int flag = laplacian_factor(r_v, Fa, Fb, Fc, w_face);
    EXPECT_TRUE(w_face.isApprox(w_face_true, 1e-7));
}
