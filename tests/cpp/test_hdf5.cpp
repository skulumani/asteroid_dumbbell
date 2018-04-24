#include "eigen_hdf5.hpp"

#include <gtest/gtest.h>

#include <iostream>

TEST(TestHDF5Wrapper, CreateFile) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate); 
}

TEST(TestHDF5Wrapper, CreateGroupinFile) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate);
    HDF5::Group hf_group = hf_file.create_group("group");
}

TEST(TestHDF5Wrapper, FileDataSet) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate);
    Eigen::MatrixXd mat(1, 3), mat_load(1, 3);
    mat = Eigen::MatrixXd::Random(1, 3);
    hf_file.write("matrix", mat);

    hf_file.read("matrix", mat_load);

    ASSERT_TRUE(mat.isApprox(mat_load));

}

TEST(TestHDF5Wrapper, GroupDataSet) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate);
    HDF5::Group hf_group = hf_file.create_group("group");
    Eigen::MatrixXd mat(1, 3), mat_load(1, 3);
    mat = Eigen::MatrixXd::Random(1, 3);
    hf_group.write("matrix", mat);

    hf_group.read("matrix", mat_load);

    ASSERT_TRUE(mat.isApprox(mat_load));

}
