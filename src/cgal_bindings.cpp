/**
    Bindings for the CGAL module

    @author Shankar Kulumani
    @version 4 April 2018
*/

#include "cgal.hpp"
#include "lidar.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(cgal, m) {
    m.doc() = "CGAL operations on the MeshData object";
    pybind11::class_<MeshDistance, std::shared_ptr<MeshDistance>>(m, "MeshDistance")
        .def(pybind11::init<std::shared_ptr<MeshData>>(), "MeshDistance constructor",
                pybind11::arg("shared_ptr to MeshData object"))
        .def("k_nearest_neighbor", &MeshDistance::k_nearest_neighbor, "Find k nearest neighboring vertices",
                pybind11::arg("pt"), pybind11::arg("k"))
        .def("update_mesh", &MeshDistance::update_mesh, "Update the mesh",
                pybind11::arg("mesh"));

    pybind11::class_<RayCaster, std::shared_ptr<RayCaster>>(m, "RayCaster")
        .def(pybind11::init<std::shared_ptr<MeshData>>(), "RayCaster constructor",
                pybind11::arg("shared_ptr to MeshData object"))
        .def("update_mesh", &RayCaster::update_mesh, "Update the mesh",
                pybind11::arg("mesh"))
        .def("castray", &RayCaster::castray, "Cast a ray and return the first intersections",
                pybind11::arg("psource"), pybind11::arg("ptarget"))
        .def("minimum_distance", &RayCaster::minimum_distance, "Minimum distance from point to mesh",
                pybind11::arg("pt"))
        .def("castarray", &RayCaster::castarray, "Cast many rays to the targets",
                pybind11::arg("psource"), pybind11::arg("targets"));

    pybind11::class_<Lidar, std::shared_ptr<Lidar>>(m, "Lidar")
        .def(pybind11::init<>(), "Lidar constructor")
        .def(pybind11::init<const Eigen::Ref<const Eigen::Vector3d>&,
                            const Eigen::Ref<const Eigen::Vector3d>&,
                            const Eigen::Ref<const Eigen::Vector2d>&,
                            const double&,
                            const double&,
                            const int&>(), "Eigen constructor",
                pybind11::arg("view axis"), 
                pybind11::arg("up axis"),
                pybind11::arg("fov"),
                pybind11::arg("sigma"),
                pybind11::arg("dist"),
                pybind11::arg("numbr of steps"));
        
}

