#include "lidar.hpp"

#include <cmath>

Lidar::Lidar( void ) {
	this->view_axis << 1, 0, 0;
	this->up_axis << 0, 0, 1;
	this->fov << 7, 7;
	this->sigma = 0.2;
	this->dist = 1;
	this->num_steps = 3;

	this->init();
}

void Lidar::init( void ) {
	Eigen::Vector3d right_axis;
	this->right_axis = this->view_axis.cross(up_axis);

	double H, W;
	H = std::tan(this->fov(1)/2) * this->dist;
	W = std::tan(this->fov(0)/2) * this->dist;

	// center of the view fustrum
	Eigen::Vector3d c;
	c = this->view_axis * dist;

	// corners of the fustrum
	Eigen::Matrix<double, Eigen::Dynamic, 1> hsteps(this->num_steps), wsteps(this->num_steps);
	hsteps = Eigen::VectorXd::LinSpaced(-H, H, num_steps);
	wsteps = Eigen::VectorXd::LinSpaced(-W, W, num_steps);

	// define  all the unit vectors for the sensor
	this->lidar_arr.resize(pow(num_steps, 2), 3);
	Eigen::Vector3d lidar_vec;
	for (int ii = 0; ii < hsteps.size(); ++ii) {
		for (int jj = 0; jj < wsteps.size(); ++jj) {
			lidar_vec = c + hsteps(ii) * up_axis + wsteps(jj) * right_axis;
			this->lidar_arr.row(ii *  num_steps + jj) << lidar_vec.normalized();
		}
	}
}

