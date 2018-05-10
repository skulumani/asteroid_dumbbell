#include "reconstruct.hpp"
#include "mesh.hpp"
#include "geodesic.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                                  const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                                  const Eigen::Ref<const Eigen::MatrixXd> &w_in) {

    this->vertices = v_in;
    this->faces = f_in;
    this->weights = w_in;

    // need to initialize the meshdata shared_ptr
    this->mesh = std::make_shared<MeshData>(vertices, faces);
}

// build object with only v and f
ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                                 const Eigen::Ref<const Eigen::MatrixXi> &f_in) {

    // need to save vectors to the object
    this->vertices = v_in;
    this->faces = f_in;
    
    // now define the weights
    this->weights.resize(this->vertices.rows(), 1);
    this->weights << initial_weight(this->vertices);

    // need to initialize the meshdata shared_ptr
    this->mesh = std::make_shared<MeshData>(vertices, faces);
}

ReconstructMesh::ReconstructMesh(std::shared_ptr<MeshData> mesh_in) {
    
    // save another ptr to object
    this->mesh = mesh_in;
    this->vertices = mesh_in->vertices;
    this->faces = mesh_in->faces;
    
    // set the weight to the maximum norm length of all vertices
    this->weights.resize(this->vertices.rows(), 1);
    this->weights << initial_weight(this->vertices);
}

Eigen::MatrixXd ReconstructMesh::get_verts( void ) const {
    return this->vertices;
}

Eigen::MatrixXi ReconstructMesh::get_faces( void ) const {
    return this->faces;
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
    
    double pt_radius = pt.norm();
    Eigen::VectorXd vert_radius = this->vertices.rowwise().norm();

    Eigen::Vector3d pt_uvec = pt.normalized();
    Eigen::Matrix<double, Eigen::Dynamic, 3> vert_uvec(this->vertices.rows(), 3);
    vert_uvec = this->vertices.rowwise().normalized();
    
    // compute the angular distance between the pt and each vertex
    Eigen::Matrix<double, Eigen::Dynamic, 1> delta_sigma(vert_uvec.rows(), 1);
    delta_sigma = central_angle(pt_uvec, vert_uvec);

    Eigen::Array<bool, Eigen::Dynamic, 1> region_condition(this->vertices.rows());
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
        mesh_region.row(ii) = this->vertices.row(region_index(ii));
        weight_old(ii) = this->weights(region_index(ii));
        radius_old(ii) = vert_radius(region_index(ii));
    }

    radius_new = (radius_old.array() * weight.array() + radius_meas * weight_old.array()) / (weight_old.array() + weight.array());
    weight_new = (weight_old.array() * weight.array()) / (weight_old.array() + weight.array());
    // Now update the vertices of the object/self
    for (int ii = 0; ii < region_index.size(); ++ii) {
        this->vertices.row(region_index(ii)) = radius_new(ii) * vert_uvec.row(region_index(ii));
        this->weights(region_index(ii)) = weight_new(ii);
    }
    
    // update the mesh pointer with the new vertices
    mesh->update_mesh(vertices, faces); 
}

void ReconstructMesh::update(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& pts,
        const double& max_angle) {
    std::size_t num_pts(pts.rows());
    
    for (int ii = 0; ii < num_pts; ++ii) {
        single_update(pts.row(ii), max_angle);
    }
}

void ReconstructMesh::update_meshdata( void ) {
    // update the data inside the mesh_ptr
    this->mesh->update_mesh(this->vertices, this->faces);
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

