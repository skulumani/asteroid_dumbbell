#include "eigen_hdf5.hpp"

#include <gtest/gtest.h>

#include <iostream>

TEST(TestHDF5Wrapper, CreateFile) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate); 
    H5::Group group(hf_file.file_ptr->createGroup("group"));
}

/* TEST(TestHDF5Wrapper, CreateGroupinFile) { */
/*     HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate); */
/*     HDF5::Group hf_group = hf_file.create_group("group"); */
/* } */
