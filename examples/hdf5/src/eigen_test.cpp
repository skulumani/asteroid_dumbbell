#include "eigen_hdf5.hpp"

#include <Eigen/Dense>
#include "H5Cpp.h"

#include <iostream>
#include <stdexcept>
#include <cassert>





int main() {
    // random matrix to write to hdf5
    Eigen::Matrix<double, Eigen::Dynamic, 3> mat(3, 3);
    mat << Eigen::MatrixXd::Random(3, 3);
    // open a new file
    H5::H5File hf("eigen_test.hdf5", H5F_ACC_TRUNC);
    
    save(hf, "eigen_test", mat);
    
    hf.close();

    // now load the file
    Eigen::Matrix<double, Eigen::Dynamic, 3> mat_read(3, 3);
    H5::H5File file("eigen_test.hdf5", H5F_ACC_RDONLY);
    load(file, "eigen_test", mat_read);

    std::cout << mat_read << std::endl;

    assert(mat.isApprox(mat_read));
    return 0;
}
