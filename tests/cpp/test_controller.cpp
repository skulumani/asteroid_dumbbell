#include "controller.hpp"
#include "state.hpp"
#include "utilities.hpp"
#include "geodesic.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <iostream>
#include <cmath>

// The fixture for testing class Foo.
class TestAttitudeController: public ::testing::Test {
    protected:
        // You can remove any or all of the following functions if its body
        // is empty.

        TestAttitudeController() {
            // You can do set-up work for each test here.
            // initialize the mesh
            state_ptr->pos(Eigen::VectorXd::Random(3, 1) + (Eigen::Vector3d() << 2, 2, 2).finished());
            state_ptr->vel(Eigen::VectorXd::Random(3, 1));
            R = Eigen::AngleAxisd(rd(), (Eigen::Vector3d() << 1, 0, 0).finished());
            state_ptr->att(R);
            state_ptr->ang_vel((Eigen::Vector3d() << 1, 1, 1).finished());
            state_ptr->time(rd() * 100);
        }

        virtual ~TestAttitudeController() {
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
        Eigen::Matrix<double, 3, 3> R;
        Rand_double rd{ 0, 2 * kPI };

};

TEST_F(TestAttitudeController, RotationMatrixSO3) {
    AttitudeController att_controller;
    // define the state
    att_controller.body_fixed_pointing_attitude(state_ptr);
    ASSERT_TRUE(assert_SO3(att_controller.get_Rd()));
}

TEST_F(TestAttitudeController, SatisfyKinematics) {
    AttitudeController att_controller;
    att_controller.body_fixed_pointing_attitude(state_ptr);
}

TEST_F(TestAttitudeController, XAxisAntiAlignedWithPositionVector) {
    AttitudeController att_controller;
    att_controller.body_fixed_pointing_attitude(state_ptr);
    
    double dot_product = att_controller.get_Rd().col(0).dot(state_ptr->get_pos().normalized()); 
    ASSERT_NEAR(dot_product, -1, 1e-6);
}

TEST_F(TestAttitudeController, ZAxisAlignedWithPositivePole) {
    AttitudeController att_controller;
    att_controller.body_fixed_pointing_attitude(state_ptr);

    Eigen::Matrix<double, 3, 1> bodyz_inertial, z_inertial;
    bodyz_inertial = att_controller.get_Rd().col(2);
    z_inertial << 0, 0, 1;
    
    double angle = std::acos(bodyz_inertial.dot(z_inertial));
    ASSERT_LE(angle, kPI / 2);
}

