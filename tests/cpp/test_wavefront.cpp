#include "wavefront.hpp"

#include "gtest/gtest.h"

#include <Eigen/Dense>

#include <vector>

class ReadOBJ: public ::testing::Test {
    protected:
    ReadOBJ() {
        input_stream.open(input_file);

        Ve_true << -0.5,   -0.5, -0.5,
                -0.5, -0.5, 0.5,
                -0.5, 0.5,  -0.5,
                -0.5, 0.5,  0.5,
                0.5,  -0.5, -0.5,
                0.5,  -0.5, 0.5,
                0.5,  0.5,  -0.5,
                0.5,  0.5,  0.5;

        Fe_true << 1, 7, 5,
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
        Fe_true = Fe_true.array() - 1;

    }
    
    std::vector<std::vector<double> > Vv_true {  { -0.5, -0.5, -0.5},
                                             { -0.5, -0.5, 0.5},
                                             { -0.5, 0.5, -0.5},
                                             { -0.5, 0.5, 0.5},
                                             { 0.5, -0.5, -0.5},
                                             { 0.5, -0.5, 0.5},
                                             { 0.5, 0.5, -0.5},
                                             { 0.5, 0.5, 0.5}};
    std::vector<std::vector<int> > Fv_true {
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
    Eigen::Matrix<double, 8, 3> Ve_true;
    Eigen::Matrix<int, 12, 3> Fe_true;

};

// Using the input string 
TEST_F(ReadOBJ, ReadStringAndVertexSize) {
    obj::OBJ wavefront_obj(input_file); 
    EXPECT_EQ(wavefront_obj.vertices.rows(), 8);
    EXPECT_EQ(wavefront_obj.vertices.cols(), 3);
}

TEST_F(ReadOBJ, ReadStringAndFacesSize) {
    obj::OBJ wavefront_obj(input_file); 
    EXPECT_EQ(wavefront_obj.faces.rows(), 12);
    EXPECT_EQ(wavefront_obj.faces.cols(), 3);
}

TEST_F(ReadOBJ, ReadStringAndVertexEqual) {
    obj::OBJ wavefront_obj(input_file); 
    EXPECT_TRUE(wavefront_obj.vertices.isApprox(Ve_true));
}

TEST_F(ReadOBJ, ReadStringAndFaceEqual) {
    obj::OBJ wavefront_obj(input_file); 
    EXPECT_TRUE(wavefront_obj.faces.isApprox(Fe_true));
}

// Using the input stream
TEST_F(ReadOBJ, ReadStreamAndVertexSize) {
    obj::OBJ wavefront_obj(input_stream); 
    EXPECT_EQ(wavefront_obj.vertices.rows(), 8);
    EXPECT_EQ(wavefront_obj.vertices.cols(), 3);
}

TEST_F(ReadOBJ, ReadStreamAndFacesSize) {
    obj::OBJ wavefront_obj(input_stream); 
    EXPECT_EQ(wavefront_obj.faces.rows(), 12);
    EXPECT_EQ(wavefront_obj.faces.cols(), 3);
}

TEST_F(ReadOBJ, ReadStreamAndVertexEqual) {
    obj::OBJ wavefront_obj(input_stream); 
    EXPECT_TRUE(wavefront_obj.vertices.isApprox(Ve_true));
}

TEST_F(ReadOBJ, ReadStreamAndFaceEqual) {
    obj::OBJ wavefront_obj(input_stream); 
    EXPECT_TRUE(wavefront_obj.faces.isApprox(Fe_true));
}

