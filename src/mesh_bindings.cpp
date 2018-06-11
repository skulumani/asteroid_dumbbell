#include "mesh.hpp"
#include "polyhedron.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(mesh_data, m) {
    m.doc() = "CGAL Mesh Class holding both Polyhedron and Surface Mesh instances";
    py::class_<MeshData, std::shared_ptr<MeshData>>(m, "MeshData")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&>(), "MeshData constructor",
             py::arg("vertices"), py::arg("faces"))
        .def("get_verts", &MeshData::get_verts, "Return vertices of the mesh")
        .def("get_faces", &MeshData::get_faces, "Return faces of the mesh")
        .def("update_mesh", &MeshData::update_mesh, "Update the mesh using vertices and faces",
                py::arg("vertices"), py::arg("faces"))
        .def("refine_faces_in_view", &MeshData::refine_faces_in_view, "Refine and return new face centers to view",
                py::arg("asteroid frame position"), py::arg("maximum fov angle (radians)"));

}
