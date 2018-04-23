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

#endif
