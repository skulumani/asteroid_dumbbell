#include "surface_mesher.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(surface_mesh, m) {
    m.doc() = "Implicit surface mesher with CGAL";
	py::class_<SurfMesh>(m, "SurfMesh")
		.def(py::init<const double&, const double&, const double&, const double&, const double&, const double&>(), "Ellipsoid surface mesh generator object",
			 py::arg("a"), py::arg("b"), py::arg("c"), py::arg("min_angle"), 
			 py::arg("max_radius"), py::arg("max_distance"))
		.def("verts", &SurfMesh::verts, "Return the vertices of the surface mesh")
		.def("faces", &SurfMesh::faces, "Return the faces of the surface mesh");
}
