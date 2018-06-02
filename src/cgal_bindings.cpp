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
        .def(pybind11::init<std::shared_ptr<const MeshData>>(), "RayCaster constructor",
                pybind11::arg("shared_ptr to MeshData object"))
        .def(pybind11::init<const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >&,
                            const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 3> >&>(),
             "RayCaster constructor", pybind11::arg("vertices"), pybind11::arg("faces"))
        .def("update_mesh",(void (RayCaster::*)(std::shared_ptr<const MeshData>)) &RayCaster::update_mesh, "Update the mesh",
                pybind11::arg("mesh"))
        .def("update_mesh", (void (RayCaster::*)(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >&,
                                                  const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 3> >&)) &RayCaster::update_mesh, "Update the mesh",
                pybind11::arg("vertices"), pybind11::arg("faces"))
        .def("castray", &RayCaster::castray, "Cast a ray and return the first intersections",
                pybind11::arg("psource"), pybind11::arg("ptarget"))
        .def("minimum_distance", &RayCaster::minimum_distance, "Minimum distance from point to mesh",
                pybind11::arg("pt"))
        .def("castarray", &RayCaster::castarray, "Cast many rays to the targets",
                pybind11::arg("psource"), pybind11::arg("targets"))
        .def("accelerate", &RayCaster::accelerate, "Call the distance acceleration setup")
        .def("intersection", &RayCaster::intersection, "Check for intersection between source and target",
                pybind11::arg("psource"), pybind11::arg("ptarget"));

    pybind11::class_<Lidar, std::shared_ptr<Lidar>>(m, "Lidar")
        .def(pybind11::init<>(), "Lidar constructor")
        .def("view_axis", &Lidar::view_axis, "Set the view axis",
                pybind11::arg("View axis in body frame"))
        .def("up_axis", &Lidar::up_axis, "Set the up axis",
                pybind11::arg("Up axis in the body frame"))
        .def("fov", &Lidar::fov, "Set the fov in horizontal and vertical directions",
                pybind11::arg("FOV of the sensor"))
        .def("right_axis", &Lidar::right_axis, "Set the right axis of the camera frame",
                pybind11::arg("Right axis of the sensor"))
        .def("sigma", &Lidar::sigma, "Set teh sigma value",
                pybind11::arg("Sigma of the sensor"))
        .def("dist", &Lidar::dist, "Set the distance for the sensor target",
                pybind11::arg("Distance for the raycasting"))
        .def("num_steps", &Lidar::num_steps, "Set the number of steps for FOV",
                pybind11::arg("num_steps"))
        .def("rotate_fov", &Lidar::rotate_fov, "Rotate teh FOV by a R",
                pybind11::arg("R_body2frame"))
        .def("define_targets", &Lidar::define_targets, "Define the targets for the LIDAR",
                pybind11::arg("pos"), pybind11::arg("R_b2f"), pybind11::arg("dist"))
        .def("define_target", &Lidar::define_target, "Define the target for the LIDAR",
                pybind11::arg("pos"), pybind11::arg("R_b2f"), pybind11::arg("dist"))
        .def("get_view_axis", &Lidar::get_view_axis, "Return the view axis")
        .def("get_up_axis", &Lidar::get_up_axis, "Return the up axis")
        .def("get_lidar_array", &Lidar::get_lidar_array, "Return teh LIDAR array")
        .def("get_fov", &Lidar::get_fov, "Return the FOV");
        
}

