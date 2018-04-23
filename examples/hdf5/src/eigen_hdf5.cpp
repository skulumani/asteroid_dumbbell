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


	template <typename Derived>
	bool write_rowmat(const Eigen::EigenBase<Derived> &mat, 
					  const H5::DataType * const datatype,
				      H5::DataSet *dataset,
					  const H5::DataSpace* dspace) {
        if (mat.derived().innerStride() != 1)
        {
            // inner stride != 1 is an edge case this function does not (yet) handle. (I think it
            // could by using the inner stride as the first element of mstride below. But I do
            // not have a test case to try it out, so just return false for now.) 
            return false;
        }

        assert(mat.rows() >= 0);
        assert(mat.cols() >= 0);
        assert(mat.derived().outerStride() >= 0);
        hsize_t rows = hsize_t(mat.rows());
        hsize_t cols = hsize_t(mat.cols());
        hsize_t stride = hsize_t(mat.derived().outerStride());

        // slab params for the file data
        hsize_t fstride[2] = { 1, cols };

        // slab params for the memory data
        hsize_t mstride[2] = { 1, stride };

        // slab params for both file and memory data
        hsize_t count[2] = { 1, 1 };
        hsize_t block[2] = { rows, cols };
        hsize_t start[2] = { 0, 0 };

        // memory dataspace
        hsize_t mdim[2] = { rows, stride };
        H5::DataSpace mspace(2, mdim);

        dspace->selectHyperslab(H5S_SELECT_SET, count, start, fstride, block);
        mspace.selectHyperslab(H5S_SELECT_SET, count, start, mstride, block);
        dataset->write(mat.derived().data(), *datatype, mspace, *dspace);

        return true;
    }

    template <typename Derived>
        bool write_colmat(const Eigen::EigenBase<Derived> &mat,
                const H5::DataType * const datatype,
                H5::DataSet *dataset,
                const H5::DataSpace* dspace) {
            if (mat.derived().innerStride() != 1)
            {
                // inner stride != 1 is an edge case this function does not (yet) handle. (I think it
                // could by using the inner stride as the first element of mstride below. But I do
                // not have a test case to try it out, so just return false for now.) 
                return false;
            }

            assert(mat.rows() >= 0);
            assert(mat.cols() >= 0);
            assert(mat.derived().outerStride() >= 0);
            hsize_t rows = hsize_t(mat.rows());
            hsize_t cols = hsize_t(mat.cols());
            hsize_t stride = hsize_t(mat.derived().outerStride());

            // slab params for the file data
            hsize_t fstride[2] = { 1, cols };
            hsize_t fcount[2] = { 1, 1 };
            hsize_t fblock[2] = { 1, cols };

            // slab params for the memory data
            hsize_t mstride[2] = { stride, 1 };
            hsize_t mcount[2] = { 1, 1 };
            hsize_t mblock[2] = { cols, 1 };

            // memory dataspace
            hsize_t mdim[2] = { cols, stride };
            H5::DataSpace mspace(2, mdim);

            // transpose the column major data in memory to the row major data in the file by
            // writing one row slab at a time. 
            for (hsize_t i = 0; i < rows; i++)
            {
                hsize_t fstart[2] = { i, 0 };
                hsize_t mstart[2] = { 0, i };
                dspace->selectHyperslab(H5S_SELECT_SET, fcount, fstart, fstride, fblock);
                mspace.selectHyperslab(H5S_SELECT_SET, mcount, mstart, mstride, mblock);
                dataset->write(mat.derived().data(), *datatype, mspace, *dspace);
            }
            return true;
        }

    
    template <typename Derived>
    bool read_colmat(const Eigen::DenseBase<Derived> &mat,
            const H5::DataType * const datatype,
            const H5::Attribute &dataset) {
        typename Derived::Index rows = mat.rows();
        typename Derived::Index cols = mat.cols();
        typename Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp(rows, cols);
        internal::read_data(dataset, temp.data(), *datatype);
        const_cast<Eigen::DenseBase<Derived> &>(mat) = temp;
        return true;
    }
}


// Explicit template specialization
template H5::DataSpace internal::create_dataspace<Eigen::Matrix<double, -1 , 3> >(const Eigen::EigenBase<Eigen::Matrix<double, -1, 3> >&);
template H5::DataSpace internal::create_dataspace<Eigen::Matrix<int, -1 , 3> >(const Eigen::EigenBase<Eigen::Matrix<int, -1, 3> >&);

template bool internal::write_rowmat<Eigen::Matrix<double, -1, 3> > (const Eigen::EigenBase<Eigen::Matrix<double, -1, 3> >& mat,
        const H5::DataType * const datatype,
        H5::DataSet* dataset,
        const H5::DataSpace* dspace);
template bool internal::write_rowmat<Eigen::Matrix<int, -1, 3> > (const Eigen::EigenBase<Eigen::Matrix<int, -1, 3> >& mat,
        const H5::DataType * const datatype,
        H5::DataSet* dataset,
        const H5::DataSpace* dspace);

template bool internal::write_colmat<Eigen::Matrix<double, -1, 3> > (const Eigen::EigenBase<Eigen::Matrix<double, -1, 3> >& mat,
        const H5::DataType * const datatype,
        H5::DataSet* dataset,
        const H5::DataSpace* dspace);
template bool internal::write_colmat<Eigen::Matrix<int, -1, 3> > (const Eigen::EigenBase<Eigen::Matrix<int, -1, 3> >& mat,
        const H5::DataType * const datatype,
        H5::DataSet* dataset,
        const H5::DataSpace* dspace);

template bool internal::read_colmat<Eigen::Matrix<double, -1, 3> > (const Eigen::DenseBase<Eigen::Matrix<double, -1, 3> > &mat,
        const H5::DataType * const datatype,
        const H5::Attribute &dataset);
template bool internal::read_colmat<Eigen::Matrix<int, -1, 3> > (const Eigen::DenseBase<Eigen::Matrix<int, -1, 3> > &mat,
        const H5::DataType * const datatype,
        const H5::Attribute &dataset);
