#include "controller.hpp"
#include "state.hpp"

#include <gtest/gtest.h>

#include <iostream>

// The fixture for testing class Foo.
class TestController: public ::testing::Test {
    protected:
        // You can remove any or all of the following functions if its body
        // is empty.

        TestController() {
            // You can do set-up work for each test here.
            // initialize the mesh
        }

        virtual ~TestController() {
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
        Eigen::Matrix<double, 3, 1> pos, vel;

};

TEST_F(TestController, RotationMatrixDeterminant) {
    AttitudeController att_controller;
    // define the state
    Eigen::Matrix<double, 1, 18> state;
    state = Eigen::MatrixXd::Random(1, 18);
    double current_time = 1;
    /* att_controller.body_fixed_pointing_attitude(current_time, state);g */
}
