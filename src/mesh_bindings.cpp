#include "mesh.hpp"
#include "polyhedron.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(mesh_data, m) {
    m.doc() = "CGAL Mesh Class holding both Polyhedron and Surface Mesh instances";
    py::class_<MeshData>(m, "MeshData")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&>(), "MeshData constructor",
             py::arg("vertices"), py::arg("faces"))
        .def("get_verts", &MeshData::get_verts, "Return vertices of the mesh")
        .def("get_faces", &MeshData::get_faces, "Return faces of the mesh");

}
