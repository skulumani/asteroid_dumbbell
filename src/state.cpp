#include "state.hpp"

#include <Eigen/Dense>

State::State( void ) {
    mpos.setZero(3);
    mvel.setZero(3);
    mR.setIdentity(3, 3);
    mang_vel.setZero(3);
}

// Definitions for getters of member variables
Eigen::Vector3d State::get_pos( void ) {
    return mpos;
}

Eigen::Vector3d State::get_vel( void ) {
    return mvel;
}

Eigen::Matrix<double, 3, 3> State::get_att( void ) {
    return mR;
}

Eigen::Vector3d State::get_ang_vel( void ) {
    return mang_vel;
}
