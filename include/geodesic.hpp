#ifndef GEODESIC_H
#define GEODESIC_H
/**
    Spherical Trigonometry functions for geodesic computations

    @author Shankar Kulumani
    @version 11 April 2018
*/
#include <Eigen/Dense>

const double kPI = 3.141592653589793115997963468544185161591;
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

/**
    Convert spherical coords to cartesian

    @param spherical Eigen n x 3 array of radius, latitude, and longitude in radians
    @returns cartesian Eigen n x 3 array of x, y, z components
*/
Eigen::Matrix<double, Eigen::Dynamic, 3> spherical2cartesian(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &spherical);

/**
    Convert cartesian coords to spherical

    @param cartesian Eigen n x 3 array of x, y, z coords
    @returns spherical Eigen n x 3 array of radius, latitude, longitude
*/
Eigen::Matrix<double, Eigen::Dynamic, 3> cartesian2spherical(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &cartesian);

Eigen::Matrix<double, 2, 1> course_azimuth(const Eigen::Ref<const Eigen::Matrix<double, 3, 1> > &initial_point,
                                           const Eigen::Ref<const Eigen::Matrix<double, 3, 1> > &final_point);

double deg2rad(const double &degrees);
double rad2deg(const double &radians);

#endif
