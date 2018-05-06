#include "potential.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

// Define all the stuff inside the Python module for the potential
PYBIND11_MODULE(asteroid, m) {
    m.doc() = "Asteroid potential function in C++";
    
    // Expose the MesParam class
    pybind11::class_<MeshParam, std::shared_ptr<MeshParam>>(m, "MeshParam")
        .def(pybind11::init<std::shared_ptr<MeshData>>(), "MeshParam constructor",
             pybind11::arg("shared_ptr to MeshData object"))
        .def(pybind11::init<const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >&,
                            const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 3> >& >(),
             "MeshParam constructor from arrays",
             pybind11::arg("vertices"), pybind11::arg("faces"));

    // Expose the Asteroid class
    pybind11::class_<Asteroid, std::shared_ptr<Asteroid>>(m, "Asteroid")
        .def(pybind11::init<const std::string&,
                            std::shared_ptr<MeshParam>>(),
             "Asteroid constructor from MeshParam shared_ptr",
             pybind11::arg("name"), pybind11::arg("MeshParam shared_ptr"))
        .def("get_potential", &Asteroid::get_potential, "Get the potential")
        .def("get_acceleration", &Asteroid::get_acceleration, "Get the acceleration")
        .def("get_gradient_mat", &Asteroid::get_gradient_mat, "Get the gradient matrix")
        .def("get_laplace", &Asteroid::get_laplace, "Get the laplacian")
        .def("polyhedron_potential", &Asteroid::polyhedron_potential, "Compute polyhedron potential",
                pybind11::arg("state"))
        .def("get_axes", &Asteroid::get_axes, "Return axes of asteroid");

    // Some other member functions
    m.def("face_contribution", &face_contribution);
    m.def("laplacian_factor", &laplacian_factor);
    m.def("edge_factor", &edge_factor);
}
