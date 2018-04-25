#include "eigen_hdf5.hpp"

#include <gtest/gtest.h>

#include <iostream>

TEST(TestHDF5File, CreateFile) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate); 
}

TEST(TestHDF5File, CreateGroupinFile) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate);
    HDF5::Group hf_group = hf_file.create_group("group");
}

TEST(TestHDF5File, FileDataSet) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate);
    Eigen::MatrixXd mat(1, 3), mat_load(1, 3);
    mat = Eigen::MatrixXd::Random(1, 3);
    hf_file.write("matrix", mat);

    hf_file.read("matrix", mat_load);

    ASSERT_TRUE(mat.isApprox(mat_load));

}

TEST(TestHDF5File, OpenDataSet) {
    std::shared_ptr<HDF5::File> hf_file_ptr = std::make_shared<HDF5::File>("/tmp/test.hdf5", HDF5::File::Truncate);
    Eigen::MatrixXd mat(1, 3), mat_load(1, 3);
    mat = Eigen::MatrixXd::Random(1, 3);
    hf_file_ptr->write("matrix", mat);
    
    // close the file by reset
    hf_file_ptr.reset(new HDF5::File("/tmp/test.hdf5", HDF5::File::ReadOnly));
    // now open and read the data into a dataset
    HDF5::DataSet hf_dataset = hf_file_ptr->open_dataset("matrix");
}

TEST(TestHDF5File, OpenAndReadDataSet) {
    std::shared_ptr<HDF5::File> hf_file_ptr = std::make_shared<HDF5::File>("/tmp/test.hdf5", HDF5::File::Truncate);
    Eigen::MatrixXd mat(1, 3), mat_load(1, 3);
    mat = Eigen::MatrixXd::Random(1, 3);
    hf_file_ptr->write("matrix", mat);
    
    // close the file by reset
    hf_file_ptr.reset(new HDF5::File("/tmp/test.hdf5", HDF5::File::ReadOnly));
    // now open and read dataset directly
    HDF5::DataSet hf_dataset = hf_file_ptr->read_dataset("matrix", mat_load);
    ASSERT_TRUE(mat.isApprox(mat_load));

}
TEST(TestHDF5Group, GroupDataSet) {
    HDF5::File hf_file("/tmp/test.hdf5", HDF5::File::Truncate);
    HDF5::Group hf_group = hf_file.create_group("group");
    Eigen::MatrixXd mat(1, 3), mat_load(1, 3);
    mat = Eigen::MatrixXd::Random(1, 3);
    hf_group.write("matrix", mat);

    hf_group.read("matrix", mat_load);

    ASSERT_TRUE(mat.isApprox(mat_load));

}
