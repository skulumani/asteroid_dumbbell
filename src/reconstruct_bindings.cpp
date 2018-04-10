/**
    Bindings for reconstruction module

    @author Shankar Kulumani
    @version 10 April 2018
*/
#include "reconstruct.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(reconstruct, m) {
    m.doc() = "Incremental reconstruction in C++";

    pybind11::class_<ReconstructMesh, std::shared_ptr<ReconstructMesh>>(m, "ReconstructMesh")
        .def(pybind11::init<std::shared_ptr<MeshData>>(), "ReconstructMesh constructor",
                pybind11::arg("shared_ptr to MeshData object"));
}
