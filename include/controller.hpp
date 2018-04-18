#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>


class BodyFixedPointingAttitude {
    public:
        BodyFixedPointingAttitude( void );
        virtual ~BodyFixedPointingAttitude( void ) {};
    
        void control(const double &current_time,
                     const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic> > &state);
        
        // getters for the desired attitude state
        Eigen::Matrix<double, 3, 3> get_Rd();
        Eigen::Matrix<double, 3, 3> get_Rd_dot();
        Eigen::Matrix<double, 1, 3> get_ang_vel_d();
        Eigen::Matrix<double, 1, 3> get_ang_vel_d_dot();

    private:
        Eigen::Matrix<double, 3, 3> mRd;
        Eigen::Matrrix<double, 3, 3> mRd_dot;
        Eigen::Matrix<double, 1, 3> mang_vel_d;
        Eigen::Matrix<double, 1, 3> mang_vel_d_dot;
};
#endif
