#ifndef POLYHEDRON_H
#define POLYHEDRON_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <Eigen/Dense>

/* template<typename VectorType, typename IndexType> */
/* void polyhedron_to_eigen(CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>, CGAL::Polyhedron_items_with_id_3 > &P, */
/*         Eigen::PlainObjectBase<VectorType> &V, Eigen::PlainObjectBase<IndexType> &F); */


/* void eigen_to_polyhedron(Eigen::MatrixXd &V,Eigen::MatrixXi &F, */
/*         CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>, CGAL::Polyhedron_items_with_id_3 > &P); */

template<typename PolyType>
void build_polyhedron_index(PolyType &poly);

/* void print_polyhedron_stats(CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>, CGAL::Polyhedron_items_with_id_3> &P); */

#endif
