#include "reconstruct.hpp"
#include "mesh.hpp"
#include "geodesic.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                                  const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                                  const Eigen::Ref<const Eigen::MatrixXd> &w_in) {

    this->weights = w_in;

    // need to initialize the meshdata shared_ptr
    this->mesh = std::make_shared<MeshData>(v_in, f_in);
}

// build object with only v and f
ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                                 const Eigen::Ref<const Eigen::MatrixXi> &f_in) {

    
    // now define the weights
    this->weights.resize(v_in.rows(), 1);
    this->weights << initial_weight(v_in);

    // need to initialize the meshdata shared_ptr
    this->mesh = std::make_shared<MeshData>(v_in, f_in);
}

ReconstructMesh::ReconstructMesh(std::shared_ptr<MeshData> mesh_in) {
    
    // save another ptr to object
    this->mesh = mesh_in;
    // set the weight to the maximum norm length of all vertices
    this->weights.resize(this->mesh->get_verts().rows(), 1);
    this->weights << initial_weight(mesh->get_verts());
}

Eigen::MatrixXd ReconstructMesh::get_verts( void ) const {
    return this->mesh->get_verts();
}

Eigen::MatrixXi ReconstructMesh::get_faces( void ) const {
    return this->mesh->get_faces();
}

Eigen::MatrixXd ReconstructMesh::get_weights( void ) const {
    return this->weights;
}

template<typename T>
Eigen::VectorXi vector_find(const Eigen::Ref<const T> &logical_vec) {
    Eigen::VectorXi index = Eigen::VectorXi::LinSpaced(logical_vec.size(), 0, logical_vec.size() - 1);
    index.conservativeResize(std::stable_partition(
                index.data(), index.data() + index.size(), [&logical_vec](int i){return logical_vec(i);}) - index.data());
    return index;
}

void ReconstructMesh::single_update(const Eigen::Ref<const Eigen::RowVector3d> &pt,
                                    const double &max_angle) {
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> V = mesh->get_verts();
    Eigen::Matrix<int, Eigen::Dynamic, 3> F = mesh->get_faces();

    double pt_radius = pt.norm();
    Eigen::VectorXd vert_radius = V.rowwise().norm();

    Eigen::Vector3d pt_uvec = pt.normalized();
    Eigen::Matrix<double, Eigen::Dynamic, 3> vert_uvec(V.rows(), 3);
    vert_uvec = V.rowwise().normalized();
    
    // compute the angular distance between the pt and each vertex
    Eigen::Matrix<double, Eigen::Dynamic, 1> delta_sigma(vert_uvec.rows(), 1);
    delta_sigma = central_angle(pt_uvec, vert_uvec);

    Eigen::Array<bool, Eigen::Dynamic, 1> region_condition(V.rows());
    region_condition = delta_sigma.array() < max_angle;
    
    Eigen::VectorXi region_index = vector_find<Eigen::Array<bool, Eigen::Dynamic, 1> >(region_condition);
    
    auto region_count = region_index.size();
    
    Eigen::VectorXd weight(region_count), weight_old(region_count),
        radius_old(region_count), radius_new(region_count),
        weight_new(region_count);
    double &radius_meas = pt_radius;

    Eigen::Matrix<double, Eigen::Dynamic, 3> mesh_region(region_count, 3);

    for (int ii = 0; ii < region_index.size(); ++ii) {
        weight(ii) = pow(delta_sigma(region_index(ii)) * pt_radius, 2);
        mesh_region.row(ii) = V.row(region_index(ii));
        weight_old(ii) = this->weights(region_index(ii));
        radius_old(ii) = vert_radius(region_index(ii));
    }

    radius_new = (radius_old.array() * weight.array() + radius_meas * weight_old.array()) / (weight_old.array() + weight.array());
    weight_new = (weight_old.array() * weight.array()) / (weight_old.array() + weight.array());
    // Now update the mesh->vertices of the object/self
    for (int ii = 0; ii < region_index.size(); ++ii) {
        V.row(region_index(ii)) = radius_new(ii) * vert_uvec.row(region_index(ii));
        this->weights(region_index(ii)) = weight_new(ii);
    }
    
    // update the mesh pointer with the new vertices
    std::cout << V << std::endl;
    std::cout << F << std::endl;
    /* mesh->update_mesh(V, F); */ 
}

void ReconstructMesh::update(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& pts,
        const double& max_angle) {
    std::size_t num_pts(pts.rows());
    
    for (std::size_t ii = 0; ii < num_pts; ++ii) {
        single_update(pts.row(ii), max_angle);
    }
}

void ReconstructMesh::update_meshdata( void ) {
    // update the data inside the mesh_ptr
    /* this->mesh->update_mesh(this->vertices, this->faces); */
}

// compute the initial weighting matrix given the vertices
Eigen::Matrix<double, Eigen::Dynamic, 1> initial_weight(const Eigen::Ref<const Eigen::MatrixXd> &v_in) {
    Eigen::Matrix<double, Eigen::Dynamic, 1> vert_radius(v_in.rows(), 1);
    vert_radius = v_in.rowwise().norm();
    // define the weights
    Eigen::Matrix<double, Eigen::Dynamic, 1> weights(v_in.rows(), 1);

    double max_radius = vert_radius.maxCoeff();
    weights.fill(pow(kPI * max_radius, 2));
    
    return weights;
}

