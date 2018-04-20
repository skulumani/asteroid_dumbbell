#include "controller.hpp"

#include <Eigen/Dense>

#include <memory>
#include <iostream>
#include <cassert>

// getters for variables
Eigen::Matrix<double, 3, 3> AttitudeController::get_Rd() {
    return mRd;
}

Eigen::Matrix<double, 3, 3> AttitudeController::get_Rd_dot() {
    return mRd_dot;
}

Eigen::Matrix<double, 3, 1> AttitudeController::get_ang_vel_d() {
    return mang_vel_d;
}

Eigen::Matrix<double, 3, 1> AttitudeController::get_ang_vel_d_dot() {
    return mang_vel_d_dot;
}

// TODO Write a function to take a square number of elements (4, 9, n*n) and make a
// n by n matrix
void AttitudeController::body_fixed_pointing_attitude(const double &current_time,
                                                     std::shared_ptr<State> state_in) {
    
    // extract out the elements of the state
    Eigen::Vector3d pos, vel, ang_vel;
    Eigen::Matrix<double, 3, 3> R;
    pos << state_in->get_pos();
    vel << state_in->get_vel();
    R << state_in->get_att();
    ang_vel << state_in->get_ang_vel();

    // desired attitude such that b1 points to origin/asteroid
    Eigen::Matrix<double, 3, 1> b1_des(3), b2_des(3), b3_des(3), z_axis(0, 0, 1);

    b1_des = - pos.normalized();
    b3_des = z_axis - (z_axis.dot(b1_des) * b1_des) ;
    b3_des = b3_des.normalized();
    b2_des = b3_des.cross(b1_des);
    
    // set the output to the class
    mRd << b1_des, b2_des, b3_des;
    mRd_dot.setZero(3, 3);
    mang_vel_d.setZero(3, 1);
    mang_vel_d_dot.setZero(3, 1);
    
    // TODO Add some assertions here and break if not in SO(3)
    std::cout << mRd << std::endl;
}
