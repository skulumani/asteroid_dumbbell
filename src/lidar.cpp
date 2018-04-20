#include "lidar.hpp"

#include <cmath>

#include <iostream>

Lidar::Lidar( void ) {
	mview_axis << 1, 0, 0;
	mup_axis << 0, 0, 1;
	mfov << 0.12217, 0.12217;
	msigma = 0.2;
	mdist = 1;
	mnum_steps = 3;

	this->init();
}


void Lidar::init( void ) {
	mright_axis = mview_axis.cross(mup_axis);

	double H, W;
	H = std::tan(mfov(1)/2) * mdist;
	W = std::tan(mfov(0)/2) * mdist;
    
	// center of the view fustrum
	Eigen::Vector3d c;
	c = mview_axis * mdist;

	// corners of the fustrum
	Eigen::Matrix<double, Eigen::Dynamic, 1> hsteps(mnum_steps), wsteps(mnum_steps);
	hsteps = Eigen::VectorXd::LinSpaced(mnum_steps, -H, H);
	wsteps = Eigen::VectorXd::LinSpaced(mnum_steps, -W, W);

	// define  all the unit vectors for the sensor
	mlidar_array.resize(pow(mnum_steps, 2), 3);
	Eigen::RowVector3d lidar_vec;
	for (int ii = 0; ii < hsteps.size(); ++ii) {
		for (int jj = 0; jj < wsteps.size(); ++jj) {
			lidar_vec = c + hsteps(ii) * mup_axis + wsteps(jj) * mright_axis;
			mlidar_array.row(ii *  mnum_steps + jj) << lidar_vec / lidar_vec.norm();
		}
	}

}

Eigen::Matrix<double, Eigen::Dynamic, 3> Lidar::rotate_fov(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_body2frame) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> lidar_arr(mlidar_array.rows(), 3);
    lidar_arr = (R_body2frame * mlidar_array.transpose()).transpose();
	return lidar_arr;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> Lidar::define_targets(const Eigen::Ref<const Eigen::RowVector3d> &pos,
                                                               const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_b2f,
                                                               const double &dist) {

    Eigen::Matrix<double, Eigen::Dynamic, 3> targets(mnum_steps * mnum_steps, 3);
    targets = (dist * rotate_fov(R_b2f)).rowwise() + pos;
    return targets;
}

Eigen::RowVector3d Lidar::define_target(const Eigen::Ref<const Eigen::RowVector3d> &pos,
                                       const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R_b2f,
                                       const double &dist) {
    
    Eigen::RowVector3d target;
    target = (R_b2f * (dist * mview_axis)).transpose() + pos;
    return target;
}

Eigen::Vector3d Lidar::get_view_axis() {
    return mview_axis;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> Lidar::get_lidar_array() {
    return mlidar_array;
}

Eigen::Vector3d Lidar::get_up_axis() {
    return mup_axis;
}
