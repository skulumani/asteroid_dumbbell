// Test of eigen
#include <Eigen/Dense>
/* #include <unsupported/Eigen/CXX11/Tensor> */

#include <iostream>
#include <stdlib.h>

// a - nonwriteable, b - writeable
void eigen_reference_class(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& a, Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 3> > b) {
    b = 2 * a;
}

void eigen_reference(const Eigen::Array<double, 1, 3>& a, Eigen::Array<double, 1, 3>& b) {
    b = 2*a;
}

template<typename Derived> 
void eigen_template(const Eigen::PlainObjectBase<Derived>& a, Eigen::PlainObjectBase<Derived>& b) {
    b = 2*a;
}

/* void eigen_tensor(const int & num_i, const int & num_j, const int & num_k) { */
/*     Eigen::Tensor<double, 3> tensor(num_i, num_j, num_k); */
    
/*     tensor.setZero(); */
    
/*     // loop through the tensor and set each value to a random number */
/*     for (int ii = 0; ii < num_i; ++ii) { */
/*         for (int jj = 0; jj < num_j; ++jj) { */
/*             for (int kk = 0; kk < num_k; ++kk) { */
/*                 tensor(ii, jj, kk) = rand() % 10 + 1; */
/*             } */
/*         } */
/*     } */
/*     std::cout << tensor << std::endl; */

/* } */

void eigen_fancy_indexing( void ) {
     // try fancy indexing using another Eigen Matrix/Array of integers
    Eigen::Matrix<double, 5, 1> A;
    A << 1, 2, 3, 4, 5;
    Eigen::Array<int, 2, 1> index(2);
    index << 1, 2;

    std::cout << index.unaryExpr(A) << std::endl;

}

int main()
{
    Eigen::Array<double, 1, 3> a, b, c, d;
    a << 1, 2, 3;
    eigen_reference_class(a, b);
    eigen_reference(a, c);
    eigen_template(a,  d);
    std::cout << "a : \n" << a << std::endl;
    std::cout << "b: \n" << b << std::endl;
    std::cout << "c: \n" << c << std::endl;
    std::cout << "d: \n" << d << std::endl;

    /* eigen_tensor(5, 2, 3); */
    eigen_fancy_indexing();

}
