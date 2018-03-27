#include "surface_mesher.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

template<typename D>
void sub_func(Eigen::Ref<D> v) {
    v = v * 2;
}

template<typename D>
void ref_func(Eigen::Ref<D> v) {
    sub_func(v);
}

/* void ellipsoid_mesh(const double& a_in, const double& b_in, const double& c_in, */
/*         const double& max_radius, Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 3> > V, */ 
/*         Eigen::Ref<Eigen::Array<int, Eigen::Dynamic, 3> > F) { */
    
/*     ellipsoid_surface_mesher(a_in, b_in, c_in, 10, max_radius, 3 * max_radius, V, F); */
/* } */

PYBIND11_MODULE(surface_mesh, m) {
    m.doc() = "Implicit surface mesher with CGAL";
    /* m.def("ellipsoid_mesh", &ellipsoid_mesh); */
    m.def("ref_fun", &ref_func<Eigen::Matrix<double, -1, 3> >);
}
