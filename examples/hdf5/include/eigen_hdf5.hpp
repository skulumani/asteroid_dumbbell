/**
    Simple wrapper for Eigen to/from HDF5 datasets
    
    Updated/correctd from the work of Jim Garrison
    https://github.com/garrison/eigen3-hdf5

    @author Shankar Kulumani
    @version 23 April 2018
*/

#ifndef EIGEN_HDF5_H
#define EIGEN_HDF5_H
#include <Eigen/Dense>
#include "H5Cpp.h"

#include <stdexcept>

template <typename T>
struct DatatypeSpecialization;

// floating-point types

template <>
struct DatatypeSpecialization<float> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_FLOAT;
    }
};

template <>
struct DatatypeSpecialization<double> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_DOUBLE;
    }
};

template <>
struct DatatypeSpecialization<long double> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_LDOUBLE;
    }
};

// integer types

template <>
struct DatatypeSpecialization<short> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_SHORT;
    }
};

template <>
struct DatatypeSpecialization<unsigned short> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_USHORT;
    }
};

template <>
struct DatatypeSpecialization<int> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_INT;
    }
};

template <>
struct DatatypeSpecialization<unsigned int> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_UINT;
    }
};

template <>
struct DatatypeSpecialization<long> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_LONG;
    }
};

template <>
struct DatatypeSpecialization<unsigned long> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_ULONG;
    }
};

template <>
struct DatatypeSpecialization<long long> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_LLONG;
    }
};

template <>
struct DatatypeSpecialization<unsigned long long> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_ULLONG;
    }
};

template <typename Derived>
void load (const H5::H5Location &h5group, const std::string &name, const Eigen::DenseBase<Derived> &mat);

namespace internal
{
    template <typename Derived>
    H5::DataSpace create_dataspace (const Eigen::EigenBase<Derived> &mat);

    template <typename Derived>
    bool write_rowmat(const Eigen::EigenBase<Derived> &mat,
        const H5::DataType * const datatype,
        H5::DataSet *dataset,
        const H5::DataSpace* dspace);

    template <typename Derived>
    bool write_colmat(const Eigen::EigenBase<Derived> &mat,
        const H5::DataType * const datatype,
        H5::DataSet *dataset,
        const H5::DataSpace* dspace)
    {
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

    // H5::Attribute and H5::DataSet both have similar API's, and although they
    // share a common base class, the relevant methods are not virtual.  Worst
    // of all, they take their arguments in different orders!

    template <typename Scalar>
    inline void read_data (const H5::DataSet &dataset, Scalar *data, const H5::DataType &datatype)
    {
        dataset.read(data, datatype);
    }

    template <typename Scalar>
    inline void read_data (const H5::Attribute &dataset, Scalar *data, const H5::DataType &datatype)
    {
        dataset.read(datatype, data);
    }

    // read a column major attribute; I do not know if there is an hdf routine to read an
    // attribute hyperslab, so I take the lazy way out: just read the conventional hdf
    // row major data and let eigen copy it into mat. 
    template <typename Derived>
    bool read_colmat(const Eigen::DenseBase<Derived> &mat,
        const H5::DataType * const datatype,
        const H5::Attribute &dataset)
    {
        typename Derived::Index rows = mat.rows();
        typename Derived::Index cols = mat.cols();
        typename Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp(rows, cols);
        internal::read_data(dataset, temp.data(), *datatype);
        const_cast<Eigen::DenseBase<Derived> &>(mat) = temp;
        return true;
    }

    template <typename Derived>
    bool read_colmat(const Eigen::DenseBase<Derived> &mat,
        const H5::DataType * const datatype,
        const H5::DataSet &dataset)
    {
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

        if (stride != rows)
        {
            // this function does not (yet) read into a mat that has a different stride than the
            // dataset. 
            return false;
        }

        // slab params for the file data
        hsize_t fstride[2] = { 1, cols };
        hsize_t fcount[2] = { 1, 1 };
        hsize_t fblock[2] = { 1, cols };

        // file dataspace
        hsize_t fdim[2] = { rows, cols };
        H5::DataSpace fspace(2, fdim);

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
            fspace.selectHyperslab(H5S_SELECT_SET, fcount, fstart, fstride, fblock);
            mspace.selectHyperslab(H5S_SELECT_SET, mcount, mstart, mstride, mblock);
            dataset.read(const_cast<Eigen::DenseBase<Derived> &>(mat).derived().data(), *datatype, mspace, fspace);
        }
        return true;
    }

    template <typename Derived, typename DataSet>
        void _load (const DataSet &dataset, const Eigen::DenseBase<Derived> &mat)
        {
            typedef typename Derived::Scalar Scalar;
            const H5::DataSpace dataspace = dataset.getSpace();
            const std::size_t ndims = dataspace.getSimpleExtentNdims();
            assert(ndims > 0);
            const std::size_t dimensions_size = 2;
            hsize_t dimensions[dimensions_size];
            dimensions[1] = 1; // in case it's 1D
            if (ndims > dimensions_size) {
                throw std::runtime_error("HDF5 array has too many dimensions.");
            }
            dataspace.getSimpleExtentDims(dimensions);
            const hsize_t rows = dimensions[0], cols = dimensions[1];
            const H5::DataType * const datatype = DatatypeSpecialization<Scalar>::get();
            Eigen::DenseBase<Derived> &mat_ = const_cast<Eigen::DenseBase<Derived> &>(mat);
            mat_.derived().resize(rows, cols);
            bool written = false;
            bool isRowMajor = mat.Flags & Eigen::RowMajor;
            if (isRowMajor || dimensions[0] == 1 || dimensions[1] == 1)
            {
                // mat is already row major
                typename Derived::Index istride = mat_.derived().outerStride();
                assert(istride >= 0);
                hsize_t stride = istride >= 0 ? istride : 0;
                if (stride == cols || (stride == rows && cols == 1))
                {
                    // mat has natural stride, so read directly into its data block
                    read_data(dataset, mat_.derived().data(), *datatype);
                    written = true;
                }
            }
            else 
            {
                // colmajor flag is 0 so the assert needs to check that mat is not rowmajor. 
                assert(!(mat.Flags & Eigen::RowMajor));

                written = read_colmat(mat_, datatype, dataset);
            }

            if (!written)
            {
                // dataset has not been loaded directly into mat_, so as a last resort read it into a
                // temp and copy it to mat_. (Should only need to do this when the mat_ to be loaded
                // into has an unnatural stride.) 
                Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp(rows, cols);
                internal::read_data(dataset, temp.data(), *datatype);
                mat_ = temp;
                written = true;
            }
        }
}
#endif
