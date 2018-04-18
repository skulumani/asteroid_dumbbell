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
        
        Eigen::Vector3d view_axis;
        Eigen::Vector3d up_axis;
        Eigen::Vector2d fov;
        Eigen::Vector3d right_axis;
        Eigen::Matrix<double, Eigen::Dynamic, 3> lidar_arr;
        double sigma;
        double dist;
        int num_steps;
    private:
        /** @fn void init( void )
                
            Initialize some of the Lidar parameters

            @author Shankar Kulumani
            @version 17 April 2017
        */
        void init();

};
#endif
