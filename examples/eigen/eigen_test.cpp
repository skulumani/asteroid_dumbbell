// Test of eigen
#include <iostream>
#include <Eigen/Dense>

// a - nonwriteable, b - writeable
void eigen_reference_class(const Eigen::Ref<const Eigen::Array<double, 1, 3> >& a, Eigen::Ref<Eigen::Array<double, 1, 3> > b) {
    b = 2 * a;
}

void eigen_reference(const Eigen::Array<double, 1, 3>& a, Eigen::Array<double, 1, 3>& b) {
    b = 2*a;
}

template<typename Derived> 
void eigen_template(const Eigen::PlainObjectBase<Derived>& a, Eigen::PlainObjectBase<Derived>& b) {
    b = 2*a;
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

}
