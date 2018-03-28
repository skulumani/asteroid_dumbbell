#ifndef SURFACE_MESHER_H
#define SURFACE_MESHER_H
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

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

struct SurfMesh {
	// constructors
	SurfMesh( void ) {}
	SurfMesh(const double& a_in, const double& b_in, const double& c_in,
			const double& min_angle, const double& max_radius, const double& max_distance);

	// deconstructor
	virtual ~SurfMesh( void ) {}

	Polyhedron poly;
	Eigen::MatrixXd v;
	Eigen::MatrixXi f;	
};

template<typename PolyType>
void build_polyhedron_index(PolyType &P);

template<typename PolyType, typename VectorType, typename IndexType>
void polyhedron_to_eigen(PolyType &P,
        Eigen::Ref<VectorType> V, Eigen::Ref<IndexType> F);

template<typename PolyType>
int ellipsoid_surface_mesher(const double& a_in, const double& b_in, const double& c_in,
        const double& min_angle, const double& max_radius, const double& max_distance,
        PolyType& poly);

template<typename D>
void sub_func(Eigen::Ref<D> v) {
    v = v * 2;
}

template<typename D>
void ref_func(Eigen::Ref<D> v) {
    sub_func(v);
}


#endif

