#include "eigen_hdf5.hpp"

#include <Eigen/Dense>
#include "H5Cpp.h"

#include <stdexcept>

namespace internal {
    template <typename Derived>
    H5::DataSpace create_dataspace (const Eigen::EigenBase<Derived> &mat) {

        const std::size_t dimensions_size = 2;
        const hsize_t dimensions[dimensions_size] = {
            static_cast<hsize_t>(mat.rows()),
            static_cast<hsize_t>(mat.cols())
        };
        return H5::DataSpace(dimensions_size, dimensions);
    }
}

// Explicit template specialization
template H5::DataSpace internal::create_dataspace<Eigen::Matrix<double, -1 , 3> >(const Eigen::EigenBase<Eigen::Matrix<double, -1, 3> >&);
template H5::DataSpace internal::create_dataspace<Eigen::Matrix<int, -1 , 3> >(const Eigen::EigenBase<Eigen::Matrix<int, -1, 3> >&);
