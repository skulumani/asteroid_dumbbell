#include "controller.hpp"
#include "utilities.hpp"
#include "reconstruct.hpp"
#include "geodesic.hpp"
#include "potential.hpp"

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

void AttitudeController::body_fixed_pointing_attitude(const double& time, 
        const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >& state_in) {
    
    // transform the state vector into individual elements
    Eigen::Vector3d pos, vel, ang_vel;
    Eigen::Matrix<double, 3, 3> R(3, 3);
    
    pos(0) = state_in(0);
    pos(1) = state_in(1);
    pos(2) = state_in(2);
    
    vel(0) = state_in(3);
    vel(1) = state_in(4);
    vel(2) = state_in(5);

    R(0, 0) = state_in(6);
    R(0, 1) = state_in(7);
    R(0, 2) = state_in(8);

    R(1, 0) = state_in(9);
    R(1, 1) = state_in(10);
    R(1, 2) = state_in(11);

    R(2, 0) = state_in(12);
    R(2, 1) = state_in(13);
    R(2, 2) = state_in(14);

    ang_vel(0) = state_in(15);
    ang_vel(1) = state_in(16);
    ang_vel(2) = state_in(17);

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

void TranslationController::inertial_fixed_state(const double& time,
        const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >& state_in,
        const Eigen::Ref<const Eigen::Matrix<double, 1, 3> >& des_pos) {
    
    mposd = des_pos.transpose();
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

double control_cost(const double& t,
                    const Eigen::Ref<const Eigen::Matrix<double, 1, 3> >& pos_des, 
                    const std::shared_ptr<Asteroid> ast_est,
                    const double& m1, const double& m2,
                    const double& max_potential) {

    Eigen::Matrix<double, 3, 3> Ra = ast_est->rot_ast2int(t);
    
    // position of COM in asteroid frame
    Eigen::Matrix<double, 1, 3> z = (Ra.transpose() * pos_des.transpose()).transpose();
    

    ast_est->polyhedron_potential(z);

    Eigen::Matrix<double, 3, 1> control = -(Ra * ast_est->get_acceleration()) * (m1 + m2);
    
    double cost = (1.0 / max_potential) * control.transpose() * control;
    
    return cost;
}

double integrate_control_cost(const double& t,
                              const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& waypoints,
                              const std::shared_ptr<Asteroid> ast_est) {
    
    const int num_points = waypoints.rows();
    const double initial_angle = 0;
    const double final_angle = acos(waypoints.row(0).dot(waypoints.row(num_points)) / waypoints.row(0).norm() / waypoints.bottomRows(1).norm());
    const double delta_angle = final_angle / num_points;
    
    double total_cost = control_cost(t, waypoints.row(0), ast_est);

    for (int ii = 1; ii < num_points - 1; ++ii) {
        total_cost += 2 * control_cost(t, waypoints.row(ii), ast_est); 
    }
    total_cost = delta_angle / 2 * (total_cost + control_cost(t, waypoints.row(num_points), ast_est));

    return total_cost;
    
}
void TranslationController::minimize_uncertainty(const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >& state,
        std::shared_ptr<const ReconstructMesh> rmesh) {
    
    double max_weight = rmesh->get_weights().maxCoeff();
    double max_sigma = kPI;
    double max_accel; // TODO maximum possible for U_grad_norm (find potential at the suface + a little margin)

    const int num_waypoints = 5;

    double alpha(0.5); /**< Weighting factor between distance and ucnertainty */
    
    Eigen::Vector3d pos(3);
    pos(0) = state(0);
    pos(1) = state(1);
    pos(2) = state(2);
    
    Eigen::VectorXd vertex_control_cost;
    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints(num_waypoints, 3);

    for (int ii = 0; ii < rmesh->get_verts().rows(); ++ii) { 
        // TODO Compute geodesic waypoints (n) between pos and each vertex
        waypoints = sphere_waypoint(pos, rmesh->get_verts().row(ii), num_waypoints);

        // TODO Given all the waypoints find the integral of u^T R u where u = -F_1(pos_d) - F_2(pos_d)
        vertex_control_cost(ii) = integrate_control_cost(t, waypoints, ast_est); 
    }

    // Cost of each vertex as weighted sum of vertex weight and sigma of each vertex
    Eigen::VectorXd sigma = central_angle(pos.normalized(), rmesh->get_verts().rowwise().normalized());
    Eigen::VectorXd cost = - (1 - alpha) *rmesh->get_weights().array()/max_weight + alpha * sigma.array()/max_sigma ;


    // now find min index of cost
    Eigen::MatrixXd::Index min_cost_index;
    cost.minCoeff(&min_cost_index);

    Eigen::RowVector3d des_vector;

    des_vector = rmesh->get_verts().row(min_cost_index);
    // pick out the corresponding vertex of the asteroid that should be viewed
    // use current norm of position and output a position with same radius but just above the minium point
    double current_radius = pos.norm();

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

void Controller::explore_asteroid(const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >& state,
        std::shared_ptr<const ReconstructMesh> rmesh_ptr) {
    
    // choose a position to minimize the uncertainty
    minimize_uncertainty(state, rmesh_ptr);

    // build a new state_ptr with the current desired state
    std::shared_ptr<State> new_state = get_desired_state();

    // now look at the asteroid
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



