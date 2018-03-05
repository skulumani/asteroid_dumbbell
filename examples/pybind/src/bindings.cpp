#include "math.hpp"
#include "eigen.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(python_example, m) {
    m.doc() = "Python example bindings";
    m.def("add", &add);
    m.def("subtract", &subtract);
    m.def("scale_vector", &scale_vector);
}
