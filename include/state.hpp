/**
    Define the state of the vehicle

    @author Shankar Kulumani
    @version 19 April 2018
*/

#ifndef STATE_H
#define STATE_H

#include <Eigen/Dense>

#include <memory>

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
        
        inline State& accel(const Eigen::Ref<const Eigen::Vector3d> &accel) {
            maccel << accel;
            return *this;
        }

        inline State& att_dot(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_dot) {
            mR_dot << R_dot;
            return *this;
        }

        inline State& ang_vel_dot(const Eigen::Ref<const Eigen::Vector3d> &ang_vel_dot) {
            mang_vel_dot << ang_vel_dot;
            return *this;
        }

        inline State& time(const double &time_in) {
            mtime = time_in;
            return *this;
        }
        
        /** @fn Update the current state with a new state ptr
                
            Update the current state with a new state

            @param shared_ptr pointer to another state
            @returns nothin

            @author Shankar Kulumani
            @version 20 April 2018
        */
        void update_state(std::shared_ptr<State> new_state);

        // Getters to return member attributes
        Eigen::Vector3d get_pos( void ) const;
        Eigen::Vector3d get_vel( void ) const;
        Eigen::Matrix<double, 3, 3> get_att( void ) const;
        Eigen::Vector3d get_ang_vel( void ) const;

        Eigen::Vector3d get_accel( void ) const;
        Eigen::Vector3d get_ang_vel_dot( void ) const;
        Eigen::Matrix<double, 3, 3> get_att_dot( void ) const;

        Eigen::Matrix<double, 1, 18> get_state( void ) const;
        double get_time( void ) const;
    private:
        double mtime;
        Eigen::Vector3d mpos; /**< Position of the vehicle COM wrt to inertial frame and expressed in the inertial frame */
        Eigen::Vector3d mvel; /**< Velocity of teh vehicle COM wrt to inertial frame and expressed in the inertial frame */
        Eigen::Vector3d mang_vel; /**< Angular velocity of body frame wrt inertial frame defined in body frame */
        Eigen::Matrix<double, 3, 3> mR; /**< Rotation of vectors from the body frame to the inertial frame */
        Eigen::Array<double, 1, 18> mstate; /**< Big array holding all the member state variables */
        
        // Extra state variables required for controllers
        Eigen::Vector3d maccel;
        Eigen::Matrix<double, 3, 3> mR_dot;
        Eigen::Vector3d mang_vel_dot;

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
