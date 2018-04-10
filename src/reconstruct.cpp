#include "reconstruct.hpp"

#include <Eigen/Dense>

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

