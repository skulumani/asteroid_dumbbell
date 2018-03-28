#ifndef SURFACE_MESHER_H
#define SURFACE_MESHER_H

#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <Eigen/Dense>


struct SurfMesh {
	// constructors
	SurfMesh( void ) {}
	SurfMesh(const double& a_in, const double& b_in, const double& c_in,
			const double& min_angle, const double& max_radius, const double& max_distance);

	// deconstructor
	virtual ~SurfMesh( void ) {}
	// member functions
	Eigen::MatrixXd verts( void ) { return v; }
	Eigen::MatrixXi faces( void ) { return f; }
	
	CGAL::Polyhedron_3<CGAL::Surface_mesh_default_triangulation_3::Geom_traits,CGAL::Polyhedron_items_with_id_3> poly;
	Eigen::MatrixXd v;
	Eigen::MatrixXi f;	
};

#endif

