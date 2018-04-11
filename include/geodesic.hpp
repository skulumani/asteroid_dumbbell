#ifndef GEODESIC_H
#define GEODESIC_H
/**
    Spherical Trigonometry functions for geodesic computations

    @author Shankar Kulumani
    @version 11 April 2018
*/
#include <Eigen/Dense>

Eigen::VectorXd central_angle(const Eigen::Ref<const Eigen::Vector3d> &pt_uvec,
                              const Eigen::Ref<const Eigen::MatrixXd> &vert_uvec);

#endif
