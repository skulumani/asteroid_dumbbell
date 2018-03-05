#include "eigen.hpp"

#include <Eigen/Dense>

void scale_vector(Eigen::Ref<Eigen::VectorXd> v, const int scale) {
    v *= scale;
}
