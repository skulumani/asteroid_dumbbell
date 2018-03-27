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

template<typename PolyType, typename VectorType, typename IndexType>
void polyhedron_to_eigen(PolyType &P, Eigen::PlainObjectBase<VectorType> &V, Eigen::PlainObjectBase<IndexType> &F) {
    // loop over all the faces first
    
    // create some eigen arrays to store all the vertices
    const unsigned int num_v = P.size_of_vertices();
    const unsigned int num_f = P.size_of_facets();
    
    V.resize(num_v, 3);
    F.resize(num_f, 3);

    // loop and fill the eigen array
    build_polyhedron_index(P);

    std::size_t row, col;
    row = 0;
    col = 0;
    /* loop_over_facets(P); */

    // Build V
    for (Vertex_iterator vert = P.vertices_begin(); vert != P.vertices_end(); ++vert) {
        V(row, 0)  = vert->point().x();
        V(row, 1)  = vert->point().y();
        V(row, 2)  = vert->point().z();
        row += 1;
    }

    // Build F
    row = 0;
    for ( Facet_iterator face = P.facets_begin(); face != P.facets_end(); ++face) {
        Halfedge_facet_circulator vert = face->facet_begin();
        col = 0;
        do {
            F(row, col) = vert->vertex()->id();
            col += 1;
        } while( ++vert != face->facet_begin());
        row += 1;
    }
}
// Explicit specialization
template void polyhedron_to_eigen<CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);

/* template int ellipsoid_surface_mesher<CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> > >(double const&, double const&, double const&, double const&, double const&, double const&, CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >&); */
