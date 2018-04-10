#include "reconstruct.hpp"

#include <Eigen/Dense>

ReconstructMesh::ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in, 
                         const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                         const Eigen::Ref<const Eigen::MatrixXd> &weight_in ) {
    
    this->vertices = v_in;
    this->faces = f_in;
    this->weights = w_in;

}
