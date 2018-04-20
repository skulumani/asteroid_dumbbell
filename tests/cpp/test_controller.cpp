#include "controller.hpp"
#include "state.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <iostream>

// The fixture for testing class Foo.
class TestController: public ::testing::Test {
    protected:
        // You can remove any or all of the following functions if its body
        // is empty.

        TestController() {
            // You can do set-up work for each test here.
            // initialize the mesh
            state_ptr->pos((Eigen::Vector3d() << 1.5, 0, 0).finished());
            state_ptr->vel((Eigen::Vector3d() << 0, 0 ,0).finished());
            state_ptr->att(Eigen::MatrixXd::Identity(3, 3));
            state_ptr->ang_vel((Eigen::Vector3d() << 1, 1, 1).finished());
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
        // define a state
        std::shared_ptr<State> state_ptr = std::make_shared<State>();

};

TEST_F(TestController, RotationMatrixDeterminant) {
    AttitudeController att_controller;
    double current_time = 1;
    // define the state
    att_controller.body_fixed_pointing_attitude(current_time, state_ptr);
}
