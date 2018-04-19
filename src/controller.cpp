#include "controller.hpp"

#include <Eigen/Dense>

// getters for variables
Eigen::Matrix<double, 3, 3> get_Rd() {
    return mRd;
}

Eigen::Matrix<double, 3, 3> get_Rd_dot() {
    return mRd_dot;
}

Eigen::Matrix<double, 3, 1> get_ang_vel_d() {
    return mang_vel_d;
}

Eigen::Matrix<double, 3, 1> get_ang_vel_d_dot() {
    return mang_vel_d_dot;
}

void body_fixed_pointing_attitude(const double &current_time,
                                  const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic> > &state) {
    
    // extract out the elements of the state
    Eigen::Matrix<double, 3, 1> pos(3), vel(3), ang_vel(3);
    Eigen::Matrix<double, 3, 3> R(3, 3);
    pos = state.segment(0, 3);
    vel = state.segment(3, 3);
    R = state.segment(6, 9).resize(3,3);
    ang_vel = state.segment(15, 3);


    // desired attitude such that b1 points to origin/asteroid
    Eigen::Matrix<double, 3, 1> b1_des(3), b2_des(3), b3_des(3), z_axis(0, 0, 1);

    b1_des = - pos.normalized();
    b3_des = z_axis - (z_axis.dot(b1_des) * b1_des) ;
    b3_des = b3_des.normalized();
    b2_des = b3_des.cross(b1_des);

    // set the output to the class
    mRd << b1_des, b2_des, b3_des;
    mRd_dot.setZero(3, 3);
    mang_vel_d.setZero(3, 1);
    mang_vel_d_dot.setZero(3, 1);

}
