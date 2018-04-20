#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "state.hpp"

#include <Eigen/Dense>
#include <memory>

class ReconstructMesh;

class AttitudeController {
    friend class Controller;

    public:
        AttitudeController( void );
        virtual ~AttitudeController( void ) {};
    
        void body_fixed_pointing_attitude(std::shared_ptr<const State> state_in);
        
        // getters for the desired attitude state
        Eigen::Matrix<double, 3, 3> get_Rd();
        Eigen::Matrix<double, 3, 3> get_Rd_dot();
        Eigen::Matrix<double, 3, 1> get_ang_vel_d();
        Eigen::Matrix<double, 3, 1> get_ang_vel_d_dot();

    private:
        Eigen::Matrix<double, 3, 3> mRd;
        Eigen::Matrix<double, 3, 3> mRd_dot;
        Eigen::Matrix<double, 3, 1> mang_vel_d;
        Eigen::Matrix<double, 3, 1> mang_vel_d_dot;
};

class TranslationController {
    friend class Controller; 
    public:
        TranslationController( void );
        virtual ~TranslationController( void ) {};
        
        void inertial_fixed_state(std::shared_ptr<const State> state_in,
                std::shared_ptr<const State> des_state);

    private:
        Eigen::Matrix<double, 3, 1> mposd;
        Eigen::Matrix<double, 3, 1> mveld;
        Eigen::Matrix<double, 3, 1> macceld;
};

class Controller {
    public:
        Controller( void );
        virtual ~Controller( void ) {};

        std::shared_ptr<State> explore_asteroid(std::shared_ptr<const State> state, 
                std::shared_ptr<const ReconstructMesh> rmesh);
    private:
        AttitudeController att_control;
        TranslationController tran_control;
};
#endif
