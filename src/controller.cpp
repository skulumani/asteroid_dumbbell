#include "controller.hpp"
#include "utilities.hpp"
#include "reconstruct.hpp"
#include "geodesic.hpp"
#include "potential.hpp"
#include "surface_mesher.hpp"
#include "hdf5.hpp"

#include <igl/dot_row.h>
#include <igl/slice.h>

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

// TRANSLATIONCONTROLLER SHIT ***********************************************
TranslationController::TranslationController( void ) {
    mposd.setZero(3);
    mveld.setZero(3);
    macceld.setZero(3);

}

TranslationController::TranslationController(std::shared_ptr<const MeshData> meshdata_ptr,
                                             const double& max_angle) {
    
    mposd.setZero(3);
    mveld.setZero(3);
    macceld.setZero(3);

    generate_controller_mesh();
    build_controller_mesh_mapping(meshdata_ptr, max_angle);
}

void TranslationController::generate_controller_mesh( void ) {
    /* SurfMesh circle(1.0, 1.0, 1.0, 20, 0.4, 0.2); */
    /* controller_vertices = circle.get_verts().rowwise().normalized(); */
    /* controller_faces = circle.get_faces(); */

    // just load the hdf5 file
    HDF5::File hf = HDF5::File("./data/coarse_sphere.hdf5", HDF5::File::ReadOnly);
    hf.read("vertices", controller_vertices);
    hf.read("faces", controller_faces);
    controller_vertices = controller_vertices.rowwise().normalized();
}

void TranslationController::build_controller_mesh_mapping(std::shared_ptr<const MeshData> meshdata_ptr,
        const double& max_angle) {
    
    // define some references
    const Eigen::MatrixXd& highres_vertices = meshdata_ptr->vertices;
    const Eigen::MatrixXi& highres_faces = meshdata_ptr->faces;
    
    Eigen::VectorXd cos_angle(highres_vertices.rows()); 
    Eigen::VectorXd max_angle_vec(highres_vertices.rows());
    max_angle_vec.setConstant(cos(max_angle));

    Eigen::MatrixXd highres_vertices_uvec(highres_vertices.rows(), 3), 
                    controller_uvec(highres_vertices.rows(), 3);
    highres_vertices_uvec = highres_vertices.rowwise().normalized();
    
    Eigen::Array<bool, Eigen::Dynamic, 1> angle_condition(highres_vertices.rows());
    Eigen::VectorXi angle_index;
    
    mesh_mapping.resize(controller_vertices.rows());
    // loop over the low resolution mesh
    /* #pragma omp parallel for */
    for (int ii = 0; ii < controller_vertices.rows(); ++ii) {
        controller_uvec = controller_vertices.row(ii).replicate(highres_vertices.rows(), 1);
        
        // find dot product of this vector with all vectors in highres_vertices
        cos_angle = igl::dot_row(highres_vertices_uvec, controller_uvec);
        // for those less than max_angle store the index someplace
        angle_condition = cos_angle.array() > max_angle_vec.array();
        angle_index = vector_find<Eigen::Array<bool, Eigen::Dynamic, 1> >(angle_condition);
        // store in a std::vector<std::vector<Eigen::RowVectorXd> >
        mesh_mapping[ii] = angle_index;
    }
    
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

    double dot_product = waypoints.row(0).dot(waypoints.row(num_points)) / waypoints.row(0).norm() / waypoints.row(num_points).norm();
    double final_angle = 0;
    if (std::abs(dot_product + 1.0) < 1e-9) {
        final_angle = kPI;
    } else if (std::abs(dot_product - 1.0) < 1e-9) {
        final_angle = 0;
    } else {
        final_angle = acos(dot_product);
    }

    double total_cost = control_cost(t, waypoints.row(0), ast_est);
    if (waypoints.bottomRows(1).isApprox((Eigen::RowVector3d() << 0 ,0 ,0).finished())) {

    } else {
        const double delta_angle = final_angle / num_points;
        for (int ii = 1; ii < num_points - 1; ++ii) {
                total_cost += 2 * control_cost(t, waypoints.row(ii), ast_est); 
        }
        total_cost = delta_angle / 2 * (total_cost + control_cost(t, waypoints.row(num_points), ast_est));
    }

    return total_cost;
}

void TranslationController::minimize_uncertainty(const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >& state,
        std::shared_ptr<const ReconstructMesh> rmesh) {
    
    double max_weight = rmesh->get_weights().maxCoeff();
    double max_sigma = kPI;
    double max_accel; // TODO maximum possible for U_grad_norm (find potential at the suface + a little margin)

    double alpha(0.5); /**< Weighting factor between distance and ucnertainty */
    
    Eigen::Vector3d pos(3);
    pos(0) = state(0);
    pos(1) = state(1);
    pos(2) = state(2);
    
    // TODO Compute geodesic waypoints (n) between pos and each vertex
    const int num_waypoints = 5;
    for (int ii = 0; ii < rmesh->get_verts().rows(); ++ii) { 
    Eigen::Matrix<double, num_waypoints, 3> pos_d = sphere_waypoint(pos, rmesh->get_verts().row(1), num_waypoints);

    // TODO Given all the waypoints find the integral of u^T R u where u = -F_1(pos_d) - F_2(pos_d)
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

void TranslationController::minimize_uncertainty(const double& t,
        std::shared_ptr<const State> state,
        std::shared_ptr<const ReconstructMesh> rmesh,
        std::shared_ptr<Asteroid> ast_est) {
    
    double max_weight = rmesh->get_weights().maxCoeff();
    double max_sigma = kPI;
    double min_axis = ast_est->get_axes().minCoeff();
    ast_est->polyhedron_potential((Eigen::Vector3d() << 0, 0, min_axis).finished());
    double max_accel = ast_est->get_acceleration().norm();

    const int num_waypoints = 1;
    
    // weighting for each of the cost components
    double weighting_factor(0.4); /**< Weighting factor between distance and ucnertainty */
    double sigma_factor(0.5);
    double control_factor(0.1);

    Eigen::Vector3d pos(3);
    pos = state->get_pos();
    
    Eigen::VectorXd vertex_control_cost(controller_vertices.rows());
    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints(num_waypoints, 3);
    Eigen::VectorXd weight_cost(controller_vertices.rows()); 
    Eigen::VectorXd region_weight_cost;

    for (int ii = 0; ii < controller_vertices.rows(); ++ii) { 
        if (num_waypoints != 1 ) {
        waypoints = sphere_waypoint(pos, pos.norm() * controller_vertices.row(ii), num_waypoints);
        vertex_control_cost(ii) = integrate_control_cost(t, waypoints, ast_est); 
        } else {
            vertex_control_cost(ii) = control_cost(t, controller_vertices.row(ii), ast_est);
        }

        // need to extra out the vectors in this region and get the weight
        weight_cost(ii) = igl::slice(rmesh->get_weights(), mesh_mapping[ii]).sum() / mesh_mapping[ii].size(); 
    }
    // need to handle the first and last vertex
    // Cost of each vertex as weighted sum of vertex weight and sigma of each vertex
    Eigen::VectorXd sigma = central_angle(pos.normalized(), controller_vertices);
    Eigen::VectorXd cost = - weighting_factor * weight_cost.array()/max_weight 
                           + sigma_factor * sigma.array()/max_sigma
                           + control_factor * vertex_control_cost.array() / vertex_control_cost.maxCoeff() ;
    
    // now find min index of cost
    Eigen::MatrixXd::Index min_cost_index;
    cost.minCoeff(&min_cost_index);
    // now pick the best vector within the region  
    Eigen::MatrixXd region_verts;
    Eigen::VectorXd region_weight, region_cost, region_sigma;
    igl::slice(rmesh->get_verts(),mesh_mapping[min_cost_index], (Eigen::Vector3i() << 0, 1, 2).finished(), region_verts);
    region_weight = igl::slice(rmesh->get_weights(), mesh_mapping[min_cost_index]);
    region_sigma = central_angle(pos.normalized(), region_verts);

    region_cost = - 0.5 * region_weight.array() / max_weight + 0.5 * region_sigma.array() / max_sigma;

    
    // find minimum of the region
    Eigen::MatrixXd::Index min_region_index;
    region_cost.minCoeff(&min_region_index);
    // get the desired vector
    Eigen::RowVector3d des_vector;

    des_vector = region_verts.row(min_region_index);

    // pick out the corresponding vertex of the asteroid that should be viewed
    // use current norm of position and output a position with same radius but just above the minium point
    double current_radius = pos.norm();

    mposd = des_vector.normalized() * current_radius;
    mveld.setZero(3);
    macceld.setZero(3);
}

void TranslationController::minimize_uncertainty(const double& t,
        const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >& state,
        std::shared_ptr<const ReconstructMesh> rmesh,
        std::shared_ptr<Asteroid> ast_est) {
    
    double max_weight = rmesh->get_weights().maxCoeff();
    double max_sigma = kPI;
    double min_axis = ast_est->get_axes().minCoeff();
    ast_est->polyhedron_potential((Eigen::Vector3d() << 0, 0, min_axis).finished());
    double max_accel = ast_est->get_acceleration().norm();

    const int num_waypoints = 5;
    
    // weighting for each of the cost components
    double weighting_factor(0.4); /**< Weighting factor between distance and ucnertainty */
    double sigma_factor(0.5);
    double control_factor(0.1);

    Eigen::Vector3d pos(3);
    pos(0) = state(0);
    pos(1) = state(1);
    pos(2) = state(2);
    
    Eigen::VectorXd vertex_control_cost(rmesh->get_verts().rows());
    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints(num_waypoints, 3);
    
    for (int ii = 0; ii < rmesh->get_verts().rows(); ++ii) { 
        waypoints = sphere_waypoint(pos, rmesh->get_verts().row(ii), num_waypoints);
        vertex_control_cost(ii) = integrate_control_cost(t, waypoints, ast_est); 
    }

    // need to handle the first and last vertex
    // Cost of each vertex as weighted sum of vertex weight and sigma of each vertex
    Eigen::VectorXd sigma = central_angle(pos.normalized(), rmesh->get_verts().rowwise().normalized());
    Eigen::VectorXd cost = - weighting_factor *rmesh->get_weights().array()/max_weight 
                           + sigma_factor * sigma.array()/max_sigma
                           + control_factor * vertex_control_cost.array() / vertex_control_cost.maxCoeff() ;
    
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

// CONTROLLER SHIT ************************************************************
Controller::Controller( void ) {
    
}

Controller::Controller(std::shared_ptr<const MeshData> meshdata_ptr,
                       const double& max_angle) : TranslationController(meshdata_ptr, max_angle) {
    // TODO need to call different constructor of base class
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

void Controller::explore_asteroid(const double& t,
                                  std::shared_ptr<const State> state_ptr,
                                  std::shared_ptr<const ReconstructMesh> rmesh_ptr,
                                  std::shared_ptr<Asteroid> ast_est_ptr) {
    
    minimize_uncertainty(t, state_ptr, rmesh_ptr,  ast_est_ptr);

    std::shared_ptr<State> new_state = get_desired_state();

    body_fixed_pointing_attitude(new_state);
}

void Controller::explore_asteroid(const double& t,
                                  const Eigen::Ref<const Eigen::Matrix<double, 1, 18> >& state,
                                  std::shared_ptr<const ReconstructMesh> rmesh_ptr,
                                  std::shared_ptr<Asteroid> ast_est_ptr) {
    minimize_uncertainty(t, state, rmesh_ptr, ast_est_ptr);

    std::shared_ptr<State> new_state = get_desired_state();

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



