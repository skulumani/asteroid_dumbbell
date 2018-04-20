/**
    Define the state of the vehicle

    @author Shankar Kulumani
    @version 19 April 2018
*/

#ifndef STATE_H
#define STATE_H

#include <Eigen/Dense>

class State {
    public:
        State( void );
        virtual ~State( void ) {};

        // named parameters idiom to set member attributes
        inline State& pos(const Eigen::Ref<const Eigen::Vector3d> &pos_in) {
            mpos << pos_in;
            return *this;
        }

        inline State& vel(const Eigen::Ref<const Eigen::Vector3d> &vel_in) {
            mvel << vel_in;
            return *this;
        }

        inline State& att(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_in) {
            mR << R_in;
            return *this;
        }

        inline State& ang_vel(const Eigen::Ref<const Eigen::Vector3d> &ang_vel_in) {
            mang_vel << ang_vel_in;
            return *this;
        }
        
        // Getters to return member attributes
        Eigen::Vector3d get_pos( void );
        Eigen::Vector3d get_vel( void );
        Eigen::Matrix<double, 3, 3> get_att( void );
        Eigen::Vector3d get_ang_vel( void );

    private:
        Eigen::Vector3d mpos, mvel, mang_vel;
        Eigen::Matrix<double, 3, 3> mR;
};
#endif
