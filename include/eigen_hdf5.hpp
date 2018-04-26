/**
    Simple wrapper for Eigen to/from HDF5 datasets
    
    Updated/correctd from the work of Jim Garrison
    https://github.com/garrison/eigen3-hdf5

    @author Shankar Kulumani
    @version 23 April 2018
*/

#ifndef EIGEN_HDF5_H
#define EIGEN_HDF5_H

#include "hdf5_eigen.hpp"
#include "hdf5_file.hpp"
#include "hdf5_group.hpp"
#include "hdf5_dataset.hpp"

#include <Eigen/Dense>
#include "H5Cpp.h"

#include <stdexcept>
#include <memory>

// TODO Turn on compression by default or maybe with a flag as well
// TODO Add documentation
// TODO Move Each class to it's own file
#endif
