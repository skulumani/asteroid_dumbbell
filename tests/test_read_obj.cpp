#include "read_obj.hpp"

#include "gtest/gtest.h"

#include <Eigen/Dense>

#include <vector>

TEST(ReadOBJ, VectorOfVectors) {
    // initialize the truth vector of vectors
    std::vector<std::vector<double> > V_true {  { -0.5, -0.5, -0.5},
                                             { -0.5, -0.5, 0.5},
                                             { -0.5, 0.5, -0.5},
                                             { -0.5, 0.5, 0.5},
                                             { 0.5, -0.5, -0.5},
                                             { 0.5, -0.5, 0.5}
                                             { 0.5, 0.5, -0.5},
                                             { 0.5, 0.5, 0.5}};
    std::vector<std::vector<double> > V_actual;
    std::vector<std::vector<int> > F_actual;
    const std::string input_file = "./integration/cube.obj";
    // read using the function
    read_flag = obj::read(input_file, V_actual, F_actual);

    // compare the two
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RETURN_ALL_TESTS();
}
