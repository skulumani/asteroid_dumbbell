#include <igl/dot_row.h>
#include <igl/readOBJ.h>

#include "gtest/gtest.h"
#include <Eigen/Dense>

#include <iostream>

TEST(TestLibigl, RowDotProduct) {
    // define to matrices
    Eigen::MatrixXd a(2, 3), b(2, 3);
    a << 1, 1, 1,
        1, 0, 0;
    b << 1, 1, 1,
        0, 1, 0;

    Eigen::MatrixXd dot_prod = igl::dot_row(a, b);
    
    ASSERT_EQ(dot_prod(0), 3);
    ASSERT_EQ(dot_prod(1), 0);

}

TEST(TestLibigl, ReadOBJ) {

    Eigen::MatrixXd V, F;
    igl::readOBJ("./integration/cube.obj", V, F);
    
    ASSERT_EQ(V.rows(), 8);
    ASSERT_EQ(F.rows(), 12);
}
