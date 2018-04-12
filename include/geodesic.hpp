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
Eigen::VectorXd central_angle(const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &pt_uvec,
                              const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &vert_uvec);

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

/**
    Course azimuth between two points on teh sphere (latitude/longitude)

    Given a start and end point on the sphere, this will find the azimuth at the initial and final point.
    The azimuth is defined as the angle from the north.

    @param initial_point Eigen 3 x 1 spherical coordinate (r, lat, long)
    @param final_point Eigen 3 x 1 spherical coordinate (r, lat, long)
    @returns azimuth Eigen 2 x 1 angle at initial and final piont in radians
*/
Eigen::Matrix<double, 1, 2> course_azimuth(const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &initial_point,
                                           const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &final_point);

double deg2rad(const double &degrees);
double rad2deg(const double &radians);

Eigen::Matrix<double, Eigen::Dynamic, 3> geodesic_waypoint(const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &initial_point,
                                                           const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &final_point);

#endif
