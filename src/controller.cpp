#include "controller.hpp"
#include "utilities.hpp"
#include "reconstruct.hpp"

#include <Eigen/Dense>

#include <memory>
#include <iostream>
#include <cassert>

AttitudeController::AttitudeController( void ) {
    mRd.setIdentity(3, 3);
    mRd_dot.setIdentity(3, 3);
    mang_vel_d.setZero(3, 1);
    mang_vel_d_dot.setZero(3, 1);
}

// getters for variables
Eigen::Matrix<double, 3, 3> AttitudeController::get_Rd() const {
    return mRd;
}

Eigen::Matrix<double, 3, 3> AttitudeController::get_Rd_dot() const { 
    return mRd_dot;
}

Eigen::Matrix<double, 3, 1> AttitudeController::get_ang_vel_d() const {
    return mang_vel_d;
}

Eigen::Matrix<double, 3, 1> AttitudeController::get_ang_vel_d_dot() const {
    return mang_vel_d_dot;
}

void AttitudeController::body_fixed_pointing_attitude(std::shared_ptr<const State> state_in) {
    
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
    
    assert(assert_SO3(mRd));
}

TranslationController::TranslationController( void ) {
    mposd.setZero(3);
    mveld.setZero(3);
    macceld.setZero(3);
}

void TranslationController::inertial_fixed_state(std::shared_ptr<const State> des_state) {
    mposd = des_state->get_pos();
    mveld.setZero(3);
    macceld.setZero(3);
}

void TranslationController::minimize_uncertainty(std::shared_ptr<const State> state,
                                                 std::shared_ptr<const ReconstructMesh> rmesh) {
    // find index of largest uncertainty (first one)

    // pick out the corresponding vertex of the asteroid that should be viewed

    // determine a xd, veld, acceld that is above the vertex
}

Eigen::Matrix<double, 3, 1> TranslationController::get_posd( void ) const {
    return mposd;
}

Eigen::Matrix<double, 3, 1> TranslationController::get_veld( void ) const {
    return mveld;
}

Eigen::Matrix<double, 3, 1> TranslationController::get_acceld( void ) const {
    return macceld;
}

Controller::Controller( void ) {
    
}

void Controller::explore_asteroid(std::shared_ptr<const State> state_ptr,
        std::shared_ptr<const ReconstructMesh> rmesh_ptr) {
    
    // choose a position to minimize the uncertainty

    // from that position make sure we're looking at the object

    // instantiate a pointer to a new state for exploration

    // update with teh calculated state data
}


