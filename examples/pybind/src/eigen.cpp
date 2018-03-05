#include "eigen.hpp"

#include <Eigen/Dense>

void scale_vector_inplace(Eigen::Ref<Eigen::VectorXd> v, const int scale) {
    v *= scale;
}

Eigen::MatrixXd scale_vector_return(Eigen::Ref<Eigen::MatrixXd> v, const int scale) {
    Eigen::MatrixXd out;
    out = v * scale;
    return out;
}
