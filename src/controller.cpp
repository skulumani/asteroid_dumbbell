#include "controller.hpp"

#include <Eigen/Dense>

// getters for variables
Eigen::Matrix<double, 3, 3> get_Rd() {
    return mRd;
}

Eigen::Matrix<double, 3, 3> get_Rd_dot() {
    return mRd_dot;
}

Eigen::Matrix<double, 1, 3> get_ang_vel_d() {
    return mang_vel_d;
}

Eigen::Matrix<double, 1, 3> get_ang_vel_d_dot() {
    return mang_vel_d_dot;
}
