// Test of eigen
#include <iostream>
#include <Eigen/Dense>


void eigen_reference_class(const Eigen::Ref<const Eigen::Array<double, 3, 1> >& a, Eigen::Ref<Eigen::Array<double, 3, 1> > b) {
    b = 2 * a;
}

void eigen_reference() {

}

void eigen_template() {

}

int main()
{
    Eigen::Array<double, 3, 1> a, b, c, d;
    a << 1, 2, 3;
    eigen_reference_class(a, b);

    std::cout << "a : \n" << a << std::endl;
    std::cout << "b: \n" << b << std::endl;
}
