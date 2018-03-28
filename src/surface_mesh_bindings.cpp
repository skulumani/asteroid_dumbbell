#include "surface_mesher.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(surface_mesh, m) {
    m.doc() = "Implicit surface mesher with CGAL";
	py::class_<SurfMesh>(m, "SurfMesh")
		.def(py::init<const double&, const double&, const double&, const double&, const double&, const double&>())
		.def("verts", &SurfMesh::verts)
		.def("faces", &SurfMesh::faces);
}
