#include "reconstruct.hpp"
#include "mesh.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                                  const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                                  const Eigen::Ref<const Eigen::MatrixXd> &w_in) {

    this->vertices = v_in;
    this->faces = f_in;
    this->weights = w_in;
}

ReconstructMesh::ReconstructMesh(std::shared_ptr<MeshData> mesh_in) {
    
    // save another ptr to object
    this->mesh = mesh_in;
    this->vertices = mesh_in->vertices;
    this->faces = mesh_in->faces;
    
    // set the weight to the maximum norm length of all vertices
    Eigen::Matrix<double, Eigen::Dynamic, 1> vert_radius(this->vertices.rows(), 1);
    this->weights.resize(this->vertices.rows(), 1);
    vert_radius = this->vertices.rowwise().norm();
    double max_radius = vert_radius.maxCoeff();
    const double PI = 3.141592653589793115997963468544185161591;
    this->weights.fill(pow(PI * max_radius, 2));
}

Eigen::MatrixXd ReconstructMesh::get_verts( void ) {
    return this->vertices;
}

Eigen::MatrixXi ReconstructMesh::get_faces( void ) {
    return this->faces;
}

Eigen::MatrixXd ReconstructMesh::get_weights( void ) {
    return this->weights;
}

template<typename T>
Eigen::VectorXi vector_find(const Eigen::Ref<const T> &logical_vec) {
    Eigen::VectorXi index = Eigen::VectorXi::LinSpaced(logical_vec.size(), 0, logical_vec.size() - 1);
    index.conservativeResize(std::stable_partition(
                index.data(), index.data() + index.size(), [&logical_vec](int i){return logical_vec(i);}) - index.data());
    return index;
}

void ReconstructMesh::update(const Eigen::Ref<const Eigen::Vector3d> &pt,
                                    const double &max_angle) {
    
    double pt_radius = pt.norm();
    Eigen::VectorXd vert_radius = this->vertices.rowwise().norm();

    Eigen::Vector3d pt_uvec = pt.normalized();
    Eigen::Matrix<double, Eigen::Dynamic, 3> vert_uvec(this->vertices.rows(), 3);
    vert_uvec = this->vertices.rowwise().normalized();
    
    // compute the angular distance between the pt and each vertex
    Eigen::Matrix<double, Eigen::Dynamic, 1> delta_sigma(vert_uvec.rows(), 1);
    delta_sigma = spherical_distance(pt_uvec, vert_uvec);

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
        /* radius_old(ii) = vert_radius(region_index(ii)); */
    }

    radius_new = (radius_old.array() * weight.array() + radius_meas * weight_old.array()) / (weight_old.array() + weight.array());

    weight_new = weight_old.array() * weight.array() / (weight_old.array() + weight.array());
    
    // Now update the vertices of the object/self
    for (int ii = 0; ii < region_index.size(); ++ii) {
        this->vertices.row(region_index(ii)) = radius_new(ii) * vert_uvec.row(region_index(ii));
        this->weights(region_index(ii)) = weight_new(region_index(ii));
    }

}

Eigen::VectorXd spherical_distance(const Eigen::Ref<const Eigen::Vector3d> &pt_uvec,
                                    const Eigen::Ref<const Eigen::MatrixXd> &vert_uvec) {

    // compute the angular distance between the pt and each vertex
    Eigen::Matrix<double, Eigen::Dynamic, 1> cross_product(vert_uvec.rows(), 1);
    Eigen::Matrix<double, Eigen::Dynamic, 1> dot_product(vert_uvec.rows(), 1);

    cross_product =  vert_uvec.rowwise().cross(pt_uvec.transpose()).rowwise().norm();
    dot_product = (vert_uvec.array().rowwise() * pt_uvec.transpose().array()).rowwise().sum();
    
    Eigen::Matrix<double, Eigen::Dynamic, 1> delta_sigma(vert_uvec.rows(), 1);
    delta_sigma = cross_product.binaryExpr(dot_product, [] (double a, double b) { return std::atan2(a,b);} );
    return delta_sigma;
}

void ReconstructMesh::update_meshdata( void ) {
    // update the data inside the mesh_ptr
    this->mesh->update_mesh(this->vertices, this->faces);
}
