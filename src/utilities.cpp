#include "utilities.hpp"

#include <Eigen/Dense>

#include <cmath>

bool assert_SO3(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R) {
    bool isSO3;
	double errO, errD;
    Eigen::Matrix<double, 3, 3> eye3;
	eye3.setIdentity();
	const double eps=1e-6;

	errO=(R.transpose()*R-eye3).norm();
	errD=pow(1-R.determinant(),2);



	if (errO > eps || errD > eps)
	{
		isSO3=false;
	}
	else
	{
		isSO3=true;
	}
	return isSO3;

}
