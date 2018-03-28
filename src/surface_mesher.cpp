#include "surface_mesher.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>

#include <Eigen/Dense>


#include <stdlib.h>
#include <cmath>


typedef CGAL::Simple_cartesian<double> Kernel;
// default triangulation for surface_mesher
typedef CGAL::Surface_mesh_default_triangulation_3 Tr;

// c2t3
typedef CGAL::Complex_2_in_triangulation_3<Tr> C2t3;

typedef Tr::Geom_traits GT;
typedef GT::Sphere_3 Sphere_3;
typedef GT::Point_3 Point_3;
typedef GT::FT FT;
typedef CGAL::Polyhedron_3<GT,CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef Polyhedron::Facet_iterator          Facet_iterator;
typedef Polyhedron::Vertex_iterator         Vertex_iterator;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

typedef FT (*Function)(Point_3); // Function is a pointer with takes Point_3 as input and returns type FT

typedef CGAL::Implicit_surface_3<GT, Function> Surface_3;

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
void polyhedron_to_eigen(PolyType &P, 
        Eigen::Ref<VectorType> V, Eigen::Ref<IndexType> F) {
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

// these need to be used in the ellispoid function
double a, b, c;

FT ellipsoid_function (Point_3 p) {
    const FT x2 = (p.x()*p.x()) / (a * a);
    const FT y2 = (p.y()*p.y()) / (b * b);
    const FT z2 = (p.z()*p.z()) / (c * c);
    return x2 + y2 + z2 - 1;
}

template<typename PolyType>
int ellipsoid_surface_mesher(const double& a_in, const double& b_in, const double& c_in,
        const double& min_angle, const double& max_radius, const double& max_distance,
        PolyType& poly) {
    a = a_in;
    b = b_in;
    c = c_in;

    Tr tr; // 3D delaunay triangulation
    C2t3 c2t3 (tr); // 2D-complex in 3D-delaunay triangulation

    // define a surface
    Function surf_func;
    surf_func = &ellipsoid_function;
    Surface_3 surface(surf_func, // pointer to function
                      Sphere_3(CGAL::ORIGIN, pow(std::max({a, b, c}) + 0.01, 2.0) )); // bounding sphere
    // Make sure you input a squared radius of the bound sphere
    // define the meshing criteria
    CGAL::Surface_mesh_default_criteria_3<Tr> criteria(min_angle, // angular bound
                                                       max_radius, // raidus bound
                                                       max_distance); // distance bound
    
    // meshing surface
    CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_tag());

    CGAL::output_surface_facets_to_polyhedron(c2t3, poly); 

    return 0;
}

// SurMesh class
SurfMesh::SurfMesh (const double& a_in, const double& b_in, const double& c_in,
					const double& min_angle, const double& max_radius, const double& max_distance) {
	ellipsoid_surface_mesher(a_in, b_in, c_in, min_angle, max_radius, max_distance,
							this->poly);
		
    const unsigned int num_v = poly.size_of_vertices();
    const unsigned int num_f = poly.size_of_facets();
    
    this->v.resize(num_v, 3);
    this->f.resize(num_f, 3);

	polyhedron_to_eigen<Polyhedron, Eigen::MatrixXd, Eigen::MatrixXi>(this->poly, this->v, this->f);

}

// Explicit specialization
template void polyhedron_to_eigen<CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >&, Eigen::Ref<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0, Eigen::internal::conditional<Eigen::Matrix<double, -1, -1, 0, -1, -1>::IsVectorAtCompileTime, Eigen::InnerStride<1>, Eigen::OuterStride<-1> >::type>, Eigen::Ref<Eigen::Matrix<int, -1, -1, 0, -1, -1>, 0, Eigen::internal::conditional<Eigen::Matrix<int, -1, -1, 0, -1, -1>::IsVectorAtCompileTime, Eigen::InnerStride<1>, Eigen::OuterStride<-1> >::type>);

template int ellipsoid_surface_mesher<CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> > >(double const&, double const&, double const&, double const&, double const&, double const&, CGAL::Polyhedron_3<CGAL::Robust_circumcenter_traits_3<CGAL::Epick>, CGAL::Polyhedron_items_with_id_3, CGAL::HalfedgeDS_default, std::allocator<int> >&);

