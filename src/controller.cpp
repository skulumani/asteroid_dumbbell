#include "controller.hpp"
#include "utilities.hpp"
#include "reconstruct.hpp"
#include "geodesic.hpp"

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
    
    double max_weight = rmesh->get_weights().maxCoeff();
    double max_sigma = kPI;
    
    double alpha(0.5);
    // Cost of each vertex as weighted sum of vertex weight and sigma of each vertex
    Eigen::VectorXd sigma = central_angle(state->get_pos().normalized(), rmesh->get_verts().rowwise().normalized());
    Eigen::VectorXd cost = - (1 - alpha) *rmesh->get_weights().array()/max_weight + alpha * sigma.array()/max_sigma ;
    // now find min index of cost
    Eigen::MatrixXd::Index min_cost_index;
    cost.minCoeff(&min_cost_index);

    Eigen::RowVector3d des_vector;

    des_vector = rmesh->get_verts().row(min_cost_index);
    // pick out the corresponding vertex of the asteroid that should be viewed
    // use current norm of position and output a position with same radius but just above the minium point
    double current_radius = state->get_pos().norm();

    mposd = des_vector.normalized() * current_radius;
    mveld.setZero(3);
    macceld.setZero(3);
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
    minimize_uncertainty(state_ptr, rmesh_ptr);

    // Need a new state pointer with the updated position from above
    std::shared_ptr<State> new_state = get_desired_state();

    // from that position make sure we're looking at the object
    body_fixed_pointing_attitude(new_state);
    
}

std::shared_ptr<State> Controller::get_desired_state() {
    
    std::shared_ptr<State> state_ptr = std::make_shared<State>();

    // update teh state with the desired data
    state_ptr->pos(mposd);
    state_ptr->vel(mveld);
    state_ptr->accel(macceld);

    state_ptr->att(mRd);
    state_ptr->att_dot(mRd_dot);

    state_ptr->ang_vel(mang_vel_d);
    state_ptr->ang_vel_dot(mang_vel_d_dot);

    return state_ptr;
}



