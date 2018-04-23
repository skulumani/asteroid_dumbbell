#include "eigen_hdf5.hpp"

#include <Eigen/Dense>
#include "H5Cpp.h"

#include <iostream>
#include <stdexcept>
#include <cassert>



template <typename Derived>
void save (H5::H5Location &h5group, const std::string &name, const Eigen::EigenBase<Derived> &mat, const H5::DSetCreatPropList &plist=H5::DSetCreatPropList::DEFAULT)
{
    typedef typename Derived::Scalar Scalar;
    const H5::DataType * const datatype = DatatypeSpecialization<Scalar>::get();
    const H5::DataSpace dataspace = internal::create_dataspace(mat);
    H5::DataSet dataset = h5group.createDataSet(name, *datatype, dataspace, plist);

    bool written = false;  // flag will be true when the data has been written
    if (mat.derived().Flags & Eigen::RowMajor)
    {
        written = internal::write_rowmat(mat, datatype, &dataset, &dataspace);
    }
    else
    {
        written = internal::write_colmat(mat, datatype, &dataset, &dataspace);
    }
    
    if (!written)
    {
        // data has not yet been written, so there is nothing else to try but copy the input
        // matrix to a row major matrix and write it. 
        const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_major_mat(mat);
        dataset.write(row_major_mat.data(), *datatype);
    }
}


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
