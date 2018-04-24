#include "eigen_hdf5.hpp"

#include <gtest/gtest.h>

#include <iostream>

TEST(TestHDF5Wrapper, CreateFile) {
    HDF5::File hf_file("/tmp/test.hdf5", 1); 
}

TEST(TestHDF5Wrapper, CreateGroupinFile) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::ReadOnly);
    std::cout << HDF5::File::ReadOnly << std::endl;
}
