#include "reconstruct.hpp"

#include <Eigen/Dense>
#include <iostream>

ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in, 
                         const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                         const Eigen::Ref<const Eigen::MatrixXd> &w_in ) {
    
    this->vertices = v_in;
    this->faces = f_in;
    this->weights = w_in;

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

void ReconstructMesh::update_mesh(const Eigen::Ref<const Eigen::Vector3d> &pt,
                                    const double &max_angle) {
    
    double pt_radius = pt.norm();
    Eigen::VectorXd vert_radius = this->vertices.rowwise().norm();

    Eigen::Vector3d pt_uvec = pt.normalized();
    Eigen::Matrix<double, Eigen::Dynamic, 3> vert_uvec(this->vertices.rows(), 3);
    vert_uvec = this->vertices.rowwise().normalized();
    
    // compute the angular distance between the pt and each vertex
    Eigen::Matrix<double, Eigen::Dynamic, 1> cross_product(vert_uvec.rows(), 1);
    Eigen::Matrix<double, Eigen::Dynamic, 1> dot_product(vert_uvec.rows(), 1);

    cross_product =  vert_uvec.rowwise().cross(pt_uvec.transpose()).rowwise().norm();
    dot_product = - (vert_uvec.array().rowwise() * pt_uvec.transpose().array()).rowwise().sum();
    Eigen::Matrix<double, Eigen::Dynamic, 1> delta_sigma(vert_uvec.rows(), 1);

    delta_sigma = cross_product.binaryExpr(dot_product, [] (double a, double b) { return std::atan2(a,b);} );
    
    Eigen::Array<bool, Eigen::Dynamic, 1> region_index(this->vertices.rows());
    region_index = delta_sigma.array() < max_angle;
    
    auto region_count = region_index.count();
    
    /* Eigen::VectorXi index = Eigen::VectorXi::LinSpaced(region_index.size(), 0, region_index.size() - 1); */
    /* index.conservativeResize(std::stable_partition( */
    /*             index.data(), index.data() + index.size(), [&region_index](int i){return region_index(i);})- index.data()); */
    
    Eigen::VectorXi index = vector_find<Eigen::Array<bool, Eigen::Dynamic, 1> >(region_index);

    std::cout << region_index << std::endl;
    std::cout << index << std::endl;

    Eigen::VectorXd weight(region_count), weight_old(region_count), radius_old(region_count), radius_new(region_count), weight_new(region_count);
    
    // TODO now loop over and modify the vertices that are in region index
    for (int ii = 0; ii < delta_sigma.size(); ++ii) {
        
    }
}

Eigen::VectorXd spherical_distance(const Eigen::Ref<const Eigen::Vector3d> &pt_uvec,
                                    const Eigen::Ref<const Eigen::MatrixXd> &vert_uvec) {

    return pt_uvec;
}


