#include <Eigen/Dense>
#include <iostream>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

template<typename Derived> 
void scale_vector_template(Eigen::PlainObjectBase<Derived>& v, const int scale) {
    v  = v * scale;
}

PYBIND11_MODULE(py_template, m) {
    m.doc() = "Python example bindings with eigen templates";
    m.def("scale_vector_template", &scale_vector_template<Eigen::Matrix<double, -1, 3> >);
}

/* int main() { */
/*     Eigen::Vector3d v; */
/*     v << 1, 2, 3; */
/*     scale_vector_template(v, 1); */
/*     std::cout << v << std::endl; */
/* } */
