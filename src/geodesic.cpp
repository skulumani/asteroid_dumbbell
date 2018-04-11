#include "geodesic.hpp"

#include <Eigen/Dense>

#include <cmath>

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

Eigen::Matrix<double, Eigen::Dynamic, 3> cartesian2spherical(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &cartesian) {
    Eigen::Matrix<double, Eigen::Dynamic, 1> radius(cartesian.rows()), longitude(cartesian.rows()), latitude(cartesian.rows());
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> spherical(cartesian.rows(), 3);

    radius = cartesian.rowwise().norm();
    longitude = cartesian.col(1).binaryExpr(cartesian.col(0), [] (double a, double b) { return std::atan2(a, b); } ); 
    latitude = (cartesian.col(2).array() / radius.array()).asin(); 
    
    spherical << radius, latitude, longitude;

    return spherical;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> spherical2cartesian(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &spherical) {
    
    Eigen::Matrix<double, Eigen::Dynamic, 1> radius(spherical.rows()), latitude(spherical.rows()), longitude(spherical.rows()),
        x(spherical.rows()), y(spherical.rows()), z(spherical.rows());
    
    radius = spherical.col(0);
    latitude = spherical.col(1);
    longitude = spherical.col(2);

    x = radius.array() * latitude.array().cos() * longitude.array().cos();
    y = radius.array() * latitude.array().cos() * longitude.array().sin();
    z = radius.array() * latitude.array().sin();
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> cartesian(spherical.rows(), 3);

    cartesian << x, y, z;
    return cartesian;
}
