#ifndef GEODESIC_H
#define GEODESIC_H
/**
    Spherical Trigonometry functions for geodesic computations

    @author Shankar Kulumani
    @version 11 April 2018
*/
#include <Eigen/Dense>

/**
    Central angle between vectors on the two sphere

    This finds the central angle between two vectors on the sphere.
    The inputs should be unit vectors. 

    @param pt_uvec Eigen column vector of a single point
    @param vert_uvec Eigen array of many vectors to find the distance to
    @returns sigma Eigen column vector of the same size as vert_uvec of the 
        central angle (sigma)
*/
Eigen::VectorXd central_angle(const Eigen::Ref<const Eigen::Vector3d> &pt_uvec,
                              const Eigen::Ref<const Eigen::MatrixXd> &vert_uvec);

// convert spherical to cartesian and vice versa
Eigen::Matrix<double, Eigen::Dynamic, 3> spherical2cartesian(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &spherical);

Eigen::Matrix<double, Eigen::Dynamic, 3> cartesian2spherical(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &cartesian);

#endif
