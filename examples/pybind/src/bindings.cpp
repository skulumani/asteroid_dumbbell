#include <pybind11/pybind11.h>
#include "math.hpp"

namespace py = pybind11;

PYBIND11_MODULE(python_example, m) {
    m.doc() = "Python example bindings";
    m.def("add", &add);
    m.def("subtract", &subtract);
}
