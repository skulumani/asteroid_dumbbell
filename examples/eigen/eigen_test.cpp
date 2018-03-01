// Test of eigen
#include <iostream>
#include <Eigen/Dense>


int main()
{
    Eigen::Matrix<double, 2, 2> m;
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << m << std::endl;
}
