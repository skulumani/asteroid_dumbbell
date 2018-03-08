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

void scale_by_2(Eigen::Ref<Eigen::VectorXd> v) {
    v *= 2;
}

void build_polyhedron(Eigen::Ref<Eigen::MatrixXd> V, Eigen::Ref<Eigen::MatrixXi> F) {
    typedef CGAL::Simple_cartesian<double>     Kernel;
    typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>         Polyhedron;

    // create a polyhedron
    Polyhedron P;
    // compute and print some stats about it
    // Update the matrix V, F

    std::cout << "Created P" << std::endl;
    print_polyhedron_stats(P);
}

PYBIND11_MODULE(cgal_cpp, m) {
    m.doc() = "CGAL Bindings"; 
    m.def("scale_by_2", &scale_by_2);
    m.def("build_polyhedron", &build_polyhedron);
}
