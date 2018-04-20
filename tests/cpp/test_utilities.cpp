#include "utilities.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>

TEST(TestUtilities, TrueRotationMatrix) {
    Eigen::Matrix<double, 3, 3> eye3by3(3, 3);      
    eye3by3.setIdentity(3, 3);
    
    ASSERT_TRUE(assert_SO3(eye3by3));
}

TEST(TestUtilities, FalseRotationMatrix) {
    Eigen::Matrix<double, 3, 3> zero3by3(3, 3);
    zero3by3.setZero(3, 3);
    ASSERT_FALSE(assert_SO3(zero3by3));
}

TEST(TestUtilities, RandomNumberRange) {
    Rand_double rd(0, 10);

    for (int ii = 0; ii < 10; ++ii) {
        ASSERT_LE(rd(), 10);
    }
}
