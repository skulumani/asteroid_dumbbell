#include "polyhedron.hpp"

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

/* typedef CGAL::Simple_cartesian<double>     Kernel; */
/* typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>         Polyhedron; */
/* typedef Polyhedron::HalfedgeDS             HalfedgeDS; */
/* namespace py = pybind11; */

void scale_by_2(Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > v) {
    v *= 2;
}

// This makes a copy of the array
// TODO Use a py::array here to directly access the data instead of going to an Eigen array immediately
void build_polyhedron(Eigen::MatrixXd V, Eigen::MatrixXi F) {
    typedef CGAL::Simple_cartesian<double>     Kernel;
    typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>         Polyhedron;

    // create a polyhedron
    std::cout << "Creating initial P" << std::endl;
    Polyhedron P;

    std::cout << "Building P using V, F" << std::endl;
    eigen_to_polyhedron(V, F, P);
    // compute and print some stats about it
    // Update the matrix V, F

    print_polyhedron_stats(P);
}

PYBIND11_MODULE(cgal_cpp, m) {
    m.doc() = "CGAL Bindings"; 
    m.def("scale_by_2", &scale_by_2);
    m.def("build_polyhedron", &build_polyhedron);
}
