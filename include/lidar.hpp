/**
    LIDAR object for unit vectors used in raycasting

    @author Shankar Kulumani
    @version 17 April 2018
*/
#ifndef LIDAR_H
#define LIDAR_H

#include <Eigen/Dense>

class Lidar {
    public:
        Lidar( void );
        virtual ~Lidar( void ) {}
        
        // Named parameters idiom
        inline Lidar& view_axis(const Eigen::Ref<const Eigen::Vector3d> &view_axis_in) {
            mview_axis << view_axis_in;
            init();
            return *this;
        }
        
        inline Lidar& up_axis(const Eigen::Ref<const Eigen::Vector3d> &up_axis_in) {
            mup_axis << up_axis_in;
            init();
            return *this;
        }

        inline Lidar& fov(const Eigen::Ref<const Eigen::Vector2d> &fov_in) {
            mfov << fov_in;
            init();
            return *this;
        }
        
        inline Lidar& right_axis(const Eigen::Ref<const Eigen::Vector3d> &right_axis_in) {
            mright_axis << right_axis_in;
            init();
            return *this;
        }

        inline Lidar& sigma(const double &sigma_in) {
            msigma = sigma_in;
            init();
            return *this;
        }

        inline Lidar& dist(const double &dist_in) {
            mdist = dist_in;
            init();
            return *this;
        }

        inline Lidar& num_steps(const int &num_steps_in) {
            mnum_steps = num_steps_in;
            init();
            return *this;
        }
        /** @fn Lidar(const Eigen::Ref<const Eigen::Vector3d> &view_axis, const Eigen::Ref<const Eigen::Vector3d> &up_axis, const Eigen::Ref<const Eigen::Vector2d> &fov, const double &sigma, const double &dist, const int &num_steps)
         *
            Construct the lidar object from some view parameters

            @param view_axis Eigen vector3d defining the view axis in camera frame
            @param up_axis Eigen vector3d defining the up axis in the camera frame
            @param fov eigen vector2d array of horizontal and vertical field of view in radians
            @param sigma 3 sigma uncertainty of the view axis (not really used yet)
            @param dist distance to scale each unit vector
            @param num_step number of steps across the field of view to define the view arrays

            @returns just create the object

            @author Shankar Kulumani
            @version 17 April 2018
        */
        Lidar(const Eigen::Ref<const Eigen::Vector3d> &view_axis,
              const Eigen::Ref<const Eigen::Vector3d> &up_axis ,
              const Eigen::Ref<const Eigen::Vector2d> &fov, 
              const double &sigma = 0.2,
              const double &dist = 1,
              const int &num_steps = 3);
        
        /** @fn rotate_fov(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_body2frame)
            
            Rotate the FOV by a given rotation matrix. Rotates all of the lidar
            arr by the R. Only outputs the unit vectors

            @param R_body2frame Eigen (3,3) rotation matrix of body frame to inertial frame

            @returns lidar_arr return the rotated lidar array

            @author Shankar Kulumani
            @version 18 April 2018
        */
        Eigen::Matrix<double, Eigen::Dynamic, 3> rotate_fov(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_body2frame);
        
        Eigen::Matrix<double, Eigen::Dynamic, 3> define_targets(const Eigen::Ref<const Eigen::RowVector3d> &pos,
                                                                const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_b2f,
                                                                const double &dist);
        
        Eigen::RowVector3d define_target(const Eigen::Ref<const Eigen::RowVector3d> &pos,
                                                  const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_b2f,
                                                  const double &dist);
        Eigen::Vector3d get_view_axis();
        Eigen::Vector3d get_up_axis();
        Eigen::Matrix<double, Eigen::Dynamic, 3> get_lidar_array();

    private:
        /** @fn void init( void )
                
            Initialize some of the Lidar parameters

            @author Shankar Kulumani
            @version 17 April 2017
        */
        void init();

        Eigen::Vector3d mview_axis;
        Eigen::Vector3d mup_axis;
        Eigen::Vector2d mfov;
        Eigen::Vector3d mright_axis;
        Eigen::Matrix<double, Eigen::Dynamic, 3> mlidar_array;
        double msigma;
        double mdist;
        int mnum_steps;

};


#endif

