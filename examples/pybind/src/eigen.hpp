#include <Eigen/Dense>

/**
    Pass numpy array by reference and update it

    @param v Vector that is passed by reference and modified in place
    @returns v Modify the vector in place
*/
void scale_vector_inplace(Eigen::Ref<Eigen::VectorXd> v, const int scale);

Eigen::MatrixXd scale_vector_return(Eigen::Ref<Eigen::MatrixXd> v, const int scale);

template<typename Derived> 
void scale_vector_template(Eigen::PlainObjectBase<Derived>& v, const int scale);

template<typename T>
T square(T x);
