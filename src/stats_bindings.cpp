/**
    Polyhedron stats bindings

    @author Shankar Kulumani
    @version 31 May 2018
*/

#include "stats.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(stats, m) {
    m.doc() = "Statistics for Polyhedrons in C++";

    m.def("volume", (double (*)(const Eigen::Ref<const Eigen::MatrixXd>&,
                                            const Eigen::Ref<const Eigen::MatrixXi>&)) &PolyVolume::volume, "Volume of polyhedron from matrices");
}
