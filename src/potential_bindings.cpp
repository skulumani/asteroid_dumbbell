#include "potential.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

// Define all the stuff inside the Python module for the potential
PYBIND11_MODULE(asteroid, m) {
    m.doc() = "Asteroid potential function in C++";
    
    // Expose the Asteroid class
    pybind11::class_<Asteroid, std::shared_ptr<Asteroid>>(m, "Asteroid")
        .def(pybind11::init<const std::string&,
                            std::shared_ptr<ReconstructMesh>>(),
             "Asteroid constructor from ReconstructMesh shared_ptr",
             pybind11::arg("name"), pybind11::arg("ReconstructMesh shared_ptr"))
        .def(pybind11::init<const std::string&,
                             std::shared_ptr<MeshData>>(),
             "Asteroid constructor from MeshData shared_ptr",
             pybind11::arg("name"), pybind11::arg("MeshData shared_ptr"))
        .def("get_potential", &Asteroid::get_potential, "Get the potential")
        .def("get_acceleration", &Asteroid::get_acceleration, "Get the acceleration")
        .def("get_gradient_mat", &Asteroid::get_gradient_mat, "Get the gradient matrix")
        .def("get_laplace", &Asteroid::get_laplace, "Get the laplacian")
        .def("get_omega", &Asteroid::get_omega, "Get the omega rotation rate")
        .def("get_verts", &Asteroid::get_verts, "Get the vertices of the mesh")
        .def("get_faces", &Asteroid::get_faces, "Get the faces of the mesh")
        .def("set_grav_constant", &Asteroid::set_grav_constant, "Set the gravitational constant",
                pybind11::arg("G (km/kg sec^2)"))
        .def("set_sigma", &Asteroid::set_sigma, "Set the density of the asteroid",
                pybind11::arg("Sigma (density  kg/km^3)"))
        .def("polyhedron_potential", &Asteroid::polyhedron_potential, "Compute polyhedron potential",
                pybind11::arg("state"))
        .def("get_axes", &Asteroid::get_axes, "Return axes of asteroid")
        .def("rotate_vertices", &Asteroid::rotate_vertices, "Rotate teh asteroid vertices by ROT3",
                pybind11::arg("time"))
        .def("rot_ast2int", &Asteroid::rot_ast2int, "Return the asteroid rotation asteroid to inertial",
                pybind11::arg("time"))
        .def("update_rotation", &Asteroid::update_rotation, "Rotate the asteroid and update all data",
                pybind11::arg("time"))
        .def("surface_slope", &Asteroid::surface_slope, "Compute and return the surface slope for all faces")
        .def("land_in_view", &Asteroid::land_in_view, "Find a face center with lowest slope with the current FOV",
                pybind11::arg("current position in asteroid frame"), pybind11::arg("FOV in radians"))
        .def("get_name", &Asteroid::get_name, "Get asteroid name");

}
