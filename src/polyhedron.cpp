#include "polyhedron.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <iostream>

typedef CGAL::Simple_cartesian<double>     Kernel;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>         Polyhedron;
typedef Polyhedron::Facet_iterator          Facet_iterator;
typedef Polyhedron::Vertex_iterator         Vertex_iterator;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

// TODO Add the ID for each vertex to the faces
// Extract the vertex indices and face array from a Polyhedron
// convert a given eigen V and F to a polyhedron
void polyhedron_to_eigen(Polyhedron &P) {
    // loop over all the faces first
    CGAL::set_ascii_mode(std::cout);
    std::cout << "OFF" << std::endl << P.size_of_vertices() << ' '
        << P.size_of_facets() << " 0" << std::endl;
    
    // create some eigen arrays to store all the vertices
    std::size_t num_v = P.size_of_vertices();
    std::size_t num_f = P.size_of_facets();
    // loop and fill the eigen array
    std::cout << "Looping over the vertices" << std::endl;
    std::size_t ii = 0;
    for (Vertex_iterator vert = P.vertices_begin(); vert != P.vertices_end(); ++vert) {
        vert->id() = ii++; 
    }
    ii = 0; // reset the counter
    for (Facet_iterator facet = P.facets_begin(); facet != P.facets_end(); ++facet) {
        facet->id() = ii++;
    }

    std::cout << "Printing all vertices" << std::endl;
    std::copy (P.points_begin(), P.points_end(), std::ostream_iterator<Kernel::Point_3>( std::cout, "\n"));

    std::cout << std::endl << "Printing all facet indices" << std::endl;
    for ( Facet_iterator f = P.facets_begin(); f != P.facets_end(); ++f) {
        Halfedge_facet_circulator v = f->facet_begin();
        std::cout << "Number of vertices around facet: " << CGAL::circulator_size(v) << std::endl;
        do {
            std::cout << "ID: " << v->vertex()->id() << " " << "Vertex: " << v->vertex()->point() << " ";
        } while( ++v != f->facet_begin());

        std::cout << std::endl;
    }
    //
    //
}
// given a Polyhedron convert to V, F
template<typename HDS>
void Polyhedron_builder<HDS>::operator() (HDS &hds) {

    typedef typename HDS::Vertex Vertex;
    typedef typename Vertex::Point Point;

    // create the cgal incremental builder
    CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
    // initialize with #v, #f, #half-edges (optional)
    B.begin_surface(V.rows(), F.rows());

    // add all of the vertices
    for (int ii = 0; ii < V.rows(); ++ii) {
        B.add_vertex(Point(V(ii, 0), V(ii, 1), V(ii, 2)));
    }
    // add all of the faces
    for (int ii = 0; ii < F.rows(); ++ii) {
        B.begin_facet();
        B.add_vertex_to_facet(F(ii, 0));
        B.add_vertex_to_facet(F(ii, 1));
        B.add_vertex_to_facet(F(ii, 2));
        B.end_facet();
    }
    B.end_surface();
}


// Explicit initialization of the template
template void Polyhedron_builder<CGAL::HalfedgeDS_default<CGAL::Simple_cartesian<double>, CGAL::I_Polyhedron_derived_items_3<CGAL::Polyhedron_items_with_id_3>, std::allocator<int> > >::operator()(CGAL::HalfedgeDS_default<CGAL::Simple_cartesian<double>, CGAL::I_Polyhedron_derived_items_3<CGAL::Polyhedron_items_with_id_3>, std::allocator<int> >&);
