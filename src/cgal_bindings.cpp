/**
    Bindings for the CGAL module

    @author Shankar Kulumani
    @version 4 April 2018
*/

#include "cgal.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(cgal, m) {
    m.doc() = "CGAL operations on the MeshData object";
    pybind11::class_<MeshDistance>(m, "MeshDistance")
        .def(pybind11::init<std::shared_ptr<MeshData>>(), "MeshDistance constructor",
                pybind11::arg("shared_ptr to MeshData object"))
        .def("k_nearest_neighbor", &MeshDistance::k_nearest_neighbor, "Find k nearest neighboring vertices",
                pybind11::arg("pt"), pybind11::arg("k"));
}

