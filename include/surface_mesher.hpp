/**
	Surface Mesh generation using CGAL
	
	This class allows one to geneate a surface mesh for an ellipsoid
	
	See here for examples https://doc.cgal.org/latest/Surface_mesher/index.html
	@author Shankar Kulumani
	@version 20180328
*/
#ifndef SURFACE_MESHER_H
#define SURFACE_MESHER_H

#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <Eigen/Dense>


struct SurfMesh {
	// constructors
	SurfMesh( void ) {}

	/**
		Constructor for class

		@param a_in semimajor axis in x axis
		@param b_in semimajor axis in y axis
		@param c_in semimajor axis in z axis
		@param min_angle lower bound in degree for the angles of the mesh facets
		@param radius_bound upper bound on the radii of the surface delaunay balls.
			A surface delaunay ball is the ball circumscribining a mesh facet and 
			centered on the surface	
		@param max_distance Upper bound for the distance bewtween the circumcenter 
			of a mesh facet and the center of a surface delaunay ball of this
			facet
		@returns None Object instance is updated with polyhedron, vertices, and
			faces
	*/
	SurfMesh(const double& a_in, const double& b_in, const double& c_in,
			const double& min_angle, const double& max_radius, const double& max_distance);

	// deconstructor
	virtual ~SurfMesh( void ) {}
	// member functions
	Eigen::MatrixXd get_verts( void ) { return v; }
	Eigen::MatrixXi get_faces( void ) { return f; }
	
	// Instance members - The polyhedron, vertices and faces	
	CGAL::Polyhedron_3<CGAL::Surface_mesh_default_triangulation_3::Geom_traits,CGAL::Polyhedron_items_with_id_3> poly;
	Eigen::MatrixXd v;
	Eigen::MatrixXi f;	
};

#endif

