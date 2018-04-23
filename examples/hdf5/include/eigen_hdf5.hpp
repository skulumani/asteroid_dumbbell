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

template <typename Derived>
void save (H5::H5Location &h5group, const std::string &name, 
        const Eigen::EigenBase<Derived> &mat,
        const H5::DSetCreatPropList &plist=H5::DSetCreatPropList::DEFAULT);

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
        const H5::DataSpace* dspace);

    // H5::Attribute and H5::DataSet both have similar API's, and although they
    // share a common base class, the relevant methods are not virtual.  Worst
    // of all, they take their arguments in different orders!

    template <typename Scalar>
    inline void read_data (const H5::DataSet &dataset, Scalar *data, const H5::DataType &datatype) {
        dataset.read(data, datatype);
    }

    template <typename Scalar>
    inline void read_data (const H5::Attribute &dataset, Scalar *data, const H5::DataType &datatype) {
        dataset.read(datatype, data);
    }

    // read a column major attribute; I do not know if there is an hdf routine to read an
    // attribute hyperslab, so I take the lazy way out: just read the conventional hdf
    // row major data and let eigen copy it into mat. 
    template <typename Derived>
    bool read_colmat(const Eigen::DenseBase<Derived> &mat,
        const H5::DataType * const datatype,
        const H5::Attribute &dataset);

    template <typename Derived>
    bool read_colmat(const Eigen::DenseBase<Derived> &mat,
        const H5::DataType * const datatype,
        const H5::DataSet &dataset);

    template <typename Derived, typename DataSet>
        void _load (const DataSet &dataset, const Eigen::DenseBase<Derived> &mat);
}
#endif
