#include "state.hpp"

#include <Eigen/Dense>

State::State( void ) {
    mpos.setZero(3);
    mvel.setZero(3);
    mR.setIdentity(3, 3);
    mang_vel.setZero(3);
    
    mtime = 0;
    state_to_array();
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

void State::state_to_array() {
    // add postion to the array
    mstate(0) = mpos(0);
    mstate(1) = mpos(1);
    mstate(2) = mpos(2);

    // add velocity to array
    mstate(3) = mvel(0);
    mstate(4) = mvel(1);
    mstate(5) = mvel(2);

    // add rotation matrix to state array
    mstate(6) = mR(0, 0);
    mstate(7) = mR(0, 1);
    mstate(8) = mR(0, 2);
    mstate(9) = mR(1, 0);
    mstate(10) = mR(1, 1);
    mstate(11) = mR(1, 2);
    mstate(12) = mR(2, 0);
    mstate(13) = mR(2, 1);
    mstate(14) = mR(2, 2);

    // add angular velocity
    mstate(15) = mang_vel(0);
    mstate(16) = mang_vel(1);
    mstate(17) = mang_vel(2);
}
