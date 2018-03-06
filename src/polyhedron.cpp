#include "polyhedron.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include <iostream>

typedef CGAL::Simple_cartesian<double>     Kernel;
typedef CGAL::Polyhedron_3<Kernel>         Polyhedron;
typedef Polyhedron::Facet_iterator          Facet_iterator;
typedef Polyhedron::Vertex_iterator         Vertex_iterator;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

// convert a given eigen V and F to a polyhedron
void polyhedron_to_eigen(Polyhedron &P) {
    // loop over all the faces first
    CGAL::set_ascii_mode(std::cout);
    std::cout << "OFF" << std::endl << P.size_of_vertices() << ' '
        << P.size_of_facets() << " 0" << std::endl;

    std::cout << "Printing all vertices" << std::endl;
    std::copy (P.points_begin(), P.points_end(), std::ostream_iterator<Kernel::Point_3>( std::cout, "\n"));

    std::cout << std::endl << "Printing all facet indices" << std::endl;
    for ( Facet_iterator f = P.facets_begin(); f != P.facets_end(); ++f) {
        Halfedge_facet_circulator v = f->facet_begin();
        std::cout << "Number of vertices around facet: " << CGAL::circulator_size(v) << std::endl;
        do {
            std::cout << v->vertex()->point() << " ";
        } while( ++v != f->facet_begin());

        std::cout << std::endl;
    }
    //
    //
}
// given a Polyhedron convert to V, F
