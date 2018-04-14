
cmake_minimum_required(VERSION 3.10.2)
project(boost_python)

find_package(Eigen3)
include(${EIGEN3_USE_FILE})
# Find and link all the python stuff
# Eigen is located at /usr/local/include and automatically found
add_executable(eigen_test eigen_test.cpp)

