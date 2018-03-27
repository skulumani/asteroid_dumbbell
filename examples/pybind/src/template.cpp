#include <Eigen/Dense>
#include <iostream>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

template<typename T>
void scale_vector_ref(Eigen::Ref<T> v, const int& scale) {
    v *= scale;
}

PYBIND11_MODULE(template, m) {
    m.doc() = "Python example bindings with eigen templates";
    m.def("scale_vector_ref", &scale_vector_ref<Eigen::Matrix<double, -1, -1> >);
}

/* int main() { */
/*     Eigen::Vector3d v; */
/*     v << 1, 2, 3; */
/*     scale_vector_template(v, 1); */
/*     std::cout << v << std::endl; */
/* } */
