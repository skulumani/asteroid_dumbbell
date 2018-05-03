#include "potential.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

// Define all the stuff inside the Python module for the potential
PYBIND11_MODULE(polyhedron_potential, m) {
    m.def("face_contribution", &face_contribution);
    m.def("laplacian_factor", &laplacian_factor);
    m.def("edge_factor", &edge_factor);
}
