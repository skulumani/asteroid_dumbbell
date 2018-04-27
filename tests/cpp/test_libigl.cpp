#include <igl/dot_row.h>
#include <igl/readOBJ.h>

#include "gtest/gtest.h"
#include <Eigen/Dense>

#include <iostream>

TEST(TestLibigl, RowDotProduct) {
    // define to matrices
    Eigen::Matrix<double, 2, 3> a(2, 3), b(2, 3);
    a << 1, 1, 1,
        1, 0, 0;
    b << 1, 1, 1,
        0, 1, 0;

    /* auto dot_prod = igl::dot_row(a, b); */

    std::cout << a << std::endl;
}

TEST(TestLibigl, ReadOBJ) {

    Eigen::MatrixXd V, F;
    igl::readOBJ("./integration/cube.obj", V, F);

    std::cout << V << std::endl;
}
