#include <Eigen/Dense>

/**
    Pass numpy array by reference and update it

    @param v Vector that is passed by reference and modified in place
    @returns v Modify the vector in place
*/
void scale_vector(Eigen::Ref<Eigen::VectorXd> v, const int scale);
