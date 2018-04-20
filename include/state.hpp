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
            state_to_array();
            return *this;
        }

        inline State& vel(const Eigen::Ref<const Eigen::Vector3d> &vel_in) {
            mvel << vel_in;
            state_to_array();
            return *this;
        }

        inline State& att(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_in) {
            mR << R_in;
            state_to_array();
            return *this;
        }

        inline State& ang_vel(const Eigen::Ref<const Eigen::Vector3d> &ang_vel_in) {
            mang_vel << ang_vel_in;
            state_to_array();
            return *this;
        }
        
        // Getters to return member attributes
        Eigen::Vector3d get_pos( void );
        Eigen::Vector3d get_vel( void );
        Eigen::Matrix<double, 3, 3> get_att( void );
        Eigen::Vector3d get_ang_vel( void );

    private:
        Eigen::Vector3d mpos; /**< Position of the vehicle COM wrt to inertial frame and expressed in the inertial frame */
        Eigen::Vector3d mvel; /**< Velocity of teh vehicle COM wrt to inertial frame and expressed in the inertial frame */
        Eigen::Vector3d mang_vel; /**< Angular velocity of body frame wrt inertial frame defined in body frame */
        Eigen::Matrix<double, 3, 3> mR; /**< Rotation of vectors from the body frame to the inertial frame */
        Eigen::Array<double, 1, 18> mstate; /**< Big array holding all the member state variables */
        
        /** @fn Build an array out of the member variables
                
            Turn all of the individual member variables into one giant
            row vector for the state

            @param none 
            @returns none

            @author Shankar Kulumani
            @version 20 April 2018
        */
        void state_to_array();
};
#endif
