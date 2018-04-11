#include "geodesic.hpp"

#include <Eigen/Dense>


Eigen::VectorXd central_angle(const Eigen::Ref<const Eigen::Vector3d> &pt_uvec,
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
