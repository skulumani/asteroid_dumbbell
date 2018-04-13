#include "geodesic.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

Eigen::VectorXd central_angle(const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &pt_uvec,
                                    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &vert_uvec) {

    // compute the angular distance between the pt and each vertex
    Eigen::Matrix<double, Eigen::Dynamic, 1> cross_product(vert_uvec.rows(), 1);
    Eigen::Matrix<double, Eigen::Dynamic, 1> dot_product(vert_uvec.rows(), 1);

    cross_product =  vert_uvec.rowwise().cross(pt_uvec).rowwise().norm();
    dot_product = (vert_uvec.array().rowwise() * pt_uvec.array()).rowwise().sum();
    
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

Eigen::Matrix<double, 1, 2> course_azimuth(const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &initial_point,
                                           const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &final_point) {
    double lat1, lat2, long1, long2, delta_long, alpha1, alpha2;
    
    lat1 = initial_point(1);
    long1 = initial_point(2);

    lat2 = final_point(1);
    long2 = final_point(2);
    
    delta_long = long2 - long1;
    // compute the initial and final azimuth between two spherical points on teh sphere
    alpha1 = atan2(sin(delta_long), cos(lat1) * tan(lat2) - sin(lat1) * cos(delta_long) );
    alpha2 = atan2(sin(delta_long), -cos(lat2) * tan(lat1) + sin(lat2) * cos(delta_long) );

    Eigen::Matrix<double, 2, 1> azimuth(2);
    azimuth << alpha1, alpha2;
    return azimuth;
}

double deg2rad(const double &degrees) {
    double radians = degrees * (kPI / 180.0);
    return radians;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> deg2rad(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1> > &degrees) {
    // handle eigen single dimensional vectors (row or column)
    Eigen::Matrix<double, Eigen::Dynamic, 1> radians(degrees.size());
    
    radians = degrees.array() * (kPI / 180.0 );
    return radians;
}

double rad2deg(const double &radians) {
    double degrees = radians * (180.0 / kPI);
    return degrees;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> rad2deg(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1> > &radians) {
    Eigen::Matrix<double, Eigen::Dynamic, 1> degrees(radians.size());

    degrees = radians.array() * ( 180.0 / kPI);
    return degrees;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> geodesic_waypoint(const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &initial_point,
                                                           const Eigen::Ref<const Eigen::Matrix<double, 1, 3> > &final_point,
                                                           const int &num_points) {
    
    double lat1, lat2, long1, long2, long12, alpha1, alpha2;
    
    lat1 = initial_point(1);
    long1 = initial_point(2);

    lat2 = final_point(1);
    long2 = final_point(2);
    
    long12 = long2 - long1;

    // get the initial and final azimuths
    Eigen::Matrix<double, 1, 2>  azimuth = course_azimuth(initial_point, final_point); 
    alpha1 = azimuth(0);
    alpha2 = azimuth(1);

    // find the initial azimuth at the equator node
    double alpha0 = asin(sin(alpha1) * cos(lat1));

    // find the central angle between initial and final points
    Eigen::Matrix<double, 1, 3> initial_point_cartesian = spherical2cartesian(initial_point);
    Eigen::Matrix<double, 1, 3> final_point_cartesian = spherical2cartesian(final_point);
    
    initial_point_cartesian = initial_point_cartesian.array() / initial_point_cartesian.norm();
    final_point_cartesian = final_point_cartesian.array() / final_point_cartesian.norm();

    Eigen::VectorXd sigma12 = central_angle(initial_point_cartesian, final_point_cartesian);
    
    double sigma1;
    if  ( (abs(lat1) < 1e-6) && (abs(alpha1 - kPI / 2 ) ) < 1e-6 ) {
        sigma1 = 0;
    } 
    else {
        sigma1 = atan2(tan(lat1), cos(alpha1));
    }

    double sigma2 = sigma1 + sigma12(0);
    
    double long01 = atan2(sin(alpha0) * sin(sigma1), cos(sigma1));
    double long0 = long1 - long01;
    
    Eigen::Matrix<double, Eigen::Dynamic, 1> latw(num_points), longw(num_points), sigmaw(num_points), radiusw(num_points);
    sigmaw = Eigen::VectorXd::LinSpaced(num_points, 0, 1).array() * (sigma1 + sigma2);
    
    latw = (cos(alpha0) * sigmaw.array().sin()).asin();
    longw = eigen_atan2(sin(alpha0) * sigmaw.array().sin(), sigmaw.array().cos()).array() + long0;
    radiusw.fill(initial_point(0));

    // TODO Also calculate the azimuth at the midpoints
    Eigen::Matrix<double, Eigen::Dynamic, 3> waypoints(num_points, 3);
    waypoints << radiusw, latw, longw;

    return waypoints;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> eigen_atan2(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1> > &numerator, 
                                                     const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1> > &denominator) {
    
    Eigen::Matrix<double, Eigen::Dynamic, 1> angle(numerator.size());
    angle = numerator.binaryExpr(denominator, [] (double a, double b) { return std::atan2(a,b);} );
    return angle;
}
