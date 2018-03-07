#include "wavefront.hpp"

#include "gtest/gtest.h"

#include <Eigen/Dense>

#include <vector>

class ReadOBJ: public ::testing::Test {
    protected:

        ReadOBJ() {
            input_stream.open(input_file);
        }
    std::vector<std::vector<double> > V_true {  { -0.5, -0.5, -0.5},
                                             { -0.5, -0.5, 0.5},
                                             { -0.5, 0.5, -0.5},
                                             { -0.5, 0.5, 0.5},
                                             { 0.5, -0.5, -0.5},
                                             { 0.5, -0.5, 0.5},
                                             { 0.5, 0.5, -0.5},
                                             { 0.5, 0.5, 0.5}};
    std::vector<std::vector<int> > F_true {
                                          { 1, 7, 5},
                                          { 1, 3, 7},
                                          { 1, 4, 3},
                                          { 1, 2, 4},
                                          { 3, 8, 7},
                                          { 3, 4, 8},
                                          { 5, 7, 8},
                                          { 5, 8, 6},
                                          { 1, 5, 6},
                                          { 1, 6, 2},
                                          { 2, 6, 8},
                                          { 2, 8, 4}};
    const std::string input_file = "./integration/cube.obj";
    std::ifstream input_stream;
};

// Using the input string 
TEST_F(ReadOBJ, VectorOfVectorsString) {
    std::vector<std::vector<double> > V_actual;
    std::vector<std::vector<int> > F_actual;
    // read using the function
    int read_flag = obj::read(input_file, V_actual, F_actual);
    
    ASSERT_EQ(read_flag, 0);
}

TEST_F(ReadOBJ, VectorOfVectorsSizeString) {

    std::vector<std::vector<double> > V_actual;
    std::vector<std::vector<int> > F_actual;
    // read using the function
    int read_flag = obj::read(input_file, V_actual, F_actual);
    
    // compare the two
    ASSERT_EQ(V_actual.size(), V_true.size());
    ASSERT_EQ(F_actual.size(), F_true.size());

}

TEST_F(ReadOBJ, VectorOfVectorsElementsString) {
    std::vector<std::vector<double> > V_actual;
    std::vector<std::vector<int> > F_actual;
    // read using the function
    int read_flag = obj::read(input_file, V_actual, F_actual);
    // loop over the vector and compare V
    for (int ii = 0; ii < V_actual.size(); ++ii) {
        for (int jj = 0; jj < V_actual[0].size(); ++jj) {
            EXPECT_EQ(V_actual[ii][jj], V_true[ii][jj]);
        }
    }

    for (int ii = 0; ii < F_actual.size(); ++ii) {
        for (int jj = 0; jj < F_actual[0].size(); ++jj) {
            EXPECT_EQ(F_actual[ii][jj], F_true[ii][jj]-1);
        }
    }

}

// Using the input stream
TEST_F(ReadOBJ, VectorOfVectorsStream) {
    std::vector<std::vector<double> > V_actual;
    std::vector<std::vector<int> > F_actual;
    // read using the function
    int read_flag = obj::read(input_stream, V_actual, F_actual);
    
    ASSERT_EQ(read_flag, 0);
}

TEST_F(ReadOBJ, VectorOfVectorsSizeStream) {

    std::vector<std::vector<double> > V_actual;
    std::vector<std::vector<int> > F_actual;
    // read using the function
    int read_flag = obj::read(input_stream, V_actual, F_actual);
    
    // compare the two
    ASSERT_EQ(V_actual.size(), V_true.size());
    ASSERT_EQ(F_actual.size(), F_true.size());

}

TEST_F(ReadOBJ, VectorOfVectorsElementsStream) {
    std::vector<std::vector<double> > V_actual;
    std::vector<std::vector<int> > F_actual;
    // read using the function
    int read_flag = obj::read(input_stream, V_actual, F_actual);
    // loop over the vector and compare V
    for (int ii = 0; ii < V_actual.size(); ++ii) {
        for (int jj = 0; jj < V_actual[0].size(); ++jj) {
            EXPECT_EQ(V_actual[ii][jj], V_true[ii][jj]);
        }
    }

    for (int ii = 0; ii < F_actual.size(); ++ii) {
        for (int jj = 0; jj < F_actual[0].size(); ++jj) {
            EXPECT_EQ(F_actual[ii][jj], F_true[ii][jj]-1);
        }
    }

}

class EigenOBJ: public ::testing::Test {
    protected:

        EigenOBJ() {
            input_stream.open(input_file);
        }
    std::vector<std::vector<double> > V_true {  { -0.5, -0.5, -0.5},
                                             { -0.5, -0.5, 0.5},
                                             { -0.5, 0.5, -0.5},
                                             { -0.5, 0.5, 0.5},
                                             { 0.5, -0.5, -0.5},
                                             { 0.5, -0.5, 0.5},
                                             { 0.5, 0.5, -0.5},
                                             { 0.5, 0.5, 0.5}};
    std::vector<std::vector<int> > F_true {
                                          { 1, 7, 5},
                                          { 1, 3, 7},
                                          { 1, 4, 3},
                                          { 1, 2, 4},
                                          { 3, 8, 7},
                                          { 3, 4, 8},
                                          { 5, 7, 8},
                                          { 5, 8, 6},
                                          { 1, 5, 6},
                                          { 1, 6, 2},
                                          { 2, 6, 8},
                                          { 2, 8, 4}};
    const std::string input_file = "./integration/cube.obj";
    std::ifstream input_stream;
};
// TODO Unit tests for reading from OBJ to eigen arrays
TEST_F(EigenOBJ, VectorToEigen) {
    // initialize the truth vector of vectors
    std::vector<std::vector<double> > V_vector {  { -0.5, -0.5, -0.5},
                                             { -0.5, -0.5, 0.5},
                                             { -0.5, 0.5, -0.5},
                                             { -0.5, 0.5, 0.5},
                                             { 0.5, -0.5, -0.5},
                                             { 0.5, -0.5, 0.5},
                                             { 0.5, 0.5, -0.5},
                                             { 0.5, 0.5, 0.5}};
    Eigen::Matrix3d V_array;
     V_array << -0.5,   -0.5, -0.5,
           -0.5, -0.5, 0.5,
           -0.5, 0.5,  -0.5,
           -0.5, 0.5,  0.5,
           0.5,  -0.5, -0.5,
           0.5,  -0.5, 0.5,
           0.5,  0.5,  -0.5,
           0.5,  0.5,  0.5;

    Eigen::MatrixXd V_actual;
    obj::vector_array_to_eigen(V_vector, V_actual);
    
    ASSERT_TRUE(V_actual.isApprox(V_array));
}


TEST_F(EigenOBJ, EigenArray) {
    Eigen::Matrix<double, 8, 3> V_true;
    Eigen::Matrix<int, 12, 3> F_true;

    V_true << -0.5,   -0.5, -0.5,
           -0.5, -0.5, 0.5,
           -0.5, 0.5,  -0.5,
           -0.5, 0.5,  0.5,
           0.5,  -0.5, -0.5,
           0.5,  -0.5, 0.5,
           0.5,  0.5,  -0.5,
           0.5,  0.5,  0.5;

    F_true << 1, 7, 5,
           1, 3, 7,
           1, 4, 3,
           1, 2, 4,
           3, 8, 7,
           3, 4, 8,
           5, 7, 8,
           5, 8, 6,
           1, 5, 6,
           1, 6, 2,
           2, 6, 8,
           2, 8, 4;
    F_true.array() -= 1;

    const std::string input_file = "./integration/cube.obj";
    // read using the function
    int read_flag = 0;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    read_flag = obj::read_to_eigen(input_file, V, F);
    ASSERT_TRUE(V.isApprox(V_true));
    ASSERT_TRUE(F.isApprox(F_true));
}

