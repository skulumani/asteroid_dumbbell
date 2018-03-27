#include "surface_mesher.hpp"

template<typename PolyType>
void build_polyhedron_index(PolyType &P) {
    std::size_t ii = 0;
    for (Vertex_iterator vert = P.vertices_begin(); vert != P.vertices_end(); ++vert) {
        vert->id() = ii++; 
    }
    ii = 0; // reset the counter
    for (Facet_iterator facet = P.facets_begin(); facet != P.facets_end(); ++facet) {
        facet->id() = ii++;
    }

}
// Explicit specialization
/* template void polyhedron_to_eigen<CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&); */

/* template int ellipsoid_surface_mesher<CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> > >(double const&, double const&, double const&, double const&, double const&, double const&, CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >&); */
