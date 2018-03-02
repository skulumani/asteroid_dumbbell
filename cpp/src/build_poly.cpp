#include "build_poly.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

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
