#include "potential.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(polyhedron_potential, m) {
    m.def("face_contribution_loop", &face_contribution_loop);
    m.def("laplacian_factor", &laplacian_factor);
    m.def("edge_factor", &edge_factor);
}
