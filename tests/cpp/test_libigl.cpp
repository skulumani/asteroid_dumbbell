#include <igl/dot_row.h>
#include <igl/readOBJ.h>
#include <igl/sort.h>

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

TEST(TestLibigl, SortRows) {
    // Define n x 2 array
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOBJ("./integration/cube.obj", V, F);
    
    Eigen::MatrixXi Fa(F.col(0)), Fb(F.col(1)), Fc(F.col(2));
    Eigen::MatrixXi e1_vertex_map(F.rows(), 2);
    e1_vertex_map << Fb, Fa;

    Eigen::MatrixXi e1_sorted, e1_index;
    igl::sort(e1_vertex_map, 1, true, e1_sorted, e1_index);

    std::cout << e1_vertex_map << std::endl << std::endl;

    std::cout << e1_index << std::endl;
}
