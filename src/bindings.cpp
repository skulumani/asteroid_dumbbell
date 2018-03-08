#include "polyhedron.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(cgal_cpp, m) {
    m.doc() = "CGAL Bindings"; 
}
