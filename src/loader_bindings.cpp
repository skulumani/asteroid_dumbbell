#include "loader.hpp"
#include "mesh.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

// This doesn't work in python. Some issue with static functions and returning the ptr
PYBIND11_MODULE(loader, m) {
    m.doc() = "Load an OBJ file into a MeshData object and return";
    pybind11::class_<Loader, std::shared_ptr<Loader>>(m, "Loader")
        .def_static("load", &Loader::load, "Loader constructor");
}
