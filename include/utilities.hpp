#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Dense>

bool assert_so3(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R);

#endif
