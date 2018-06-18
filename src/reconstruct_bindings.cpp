/**
    Bindings for reconstruction module

    @author Shankar Kulumani
    @version 10 April 2018
*/
#include "reconstruct.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

PYBIND11_MODULE(reconstruct, m) {
    m.doc() = "Incremental reconstruction in C++";

    pybind11::class_<ReconstructMesh, std::shared_ptr<ReconstructMesh>>(m, "ReconstructMesh")
        .def(pybind11::init<std::shared_ptr<MeshData>>(), "ReconstructMesh constructor",
                pybind11::arg("shared_ptr to MeshData object"))
        .def(pybind11::init<std::shared_ptr<MeshData>,
                const Eigen::Ref<const Eigen::VectorXd> &>(),
                "ReconstructMesh from meshdata and weights",
                pybind11::arg("meshdata pointer"), pybind11::arg("weights"))
        .def(pybind11::init<const Eigen::Ref<const Eigen::MatrixXd> &, 
                            const Eigen::Ref<const Eigen::MatrixXi> &,
                            const Eigen::Ref<const Eigen::MatrixXd> &>(),
                            "ReconstructMesh constructor from eigen/numpy arrays",
                            pybind11::arg("vertices"), pybind11::arg("faces"), pybind11::arg("weights"))
        .def("single_update", &ReconstructMesh::single_update, "Update the mesh by incorporating the pt into the mesh",
                pybind11::arg("pt"), pybind11::arg("max_angle"),
                pybind11::arg("meas_weight") = 1.0, pybind11::arg("vert_weight") = 1.0)
        .def("update", &ReconstructMesh::update, "Update the mesh given many intersections",
                pybind11::arg("pts"), pybind11::arg("max_angle"), 
                pybind11::arg("meas_weight") = 1.0, pybind11::arg("vert_weight") = 1.0)
        .def("get_verts", &ReconstructMesh::get_verts, "Get the vertices")
        .def("get_faces", &ReconstructMesh::get_faces, "Get the faces")
        .def("get_weights", &ReconstructMesh::get_weights, "Get the weights of the vertices");
}
