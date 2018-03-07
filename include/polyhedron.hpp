#ifndef BUILD_POLY_H
#define BUILD_POLY_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <Eigen/Dense>

// declaration for the polyhedron builder
template<typename HDS> 
class Polyhedron_builder : public CGAL::Modifier_base<HDS> {
    public:
        Eigen::MatrixXd &V;
        Eigen::MatrixXi &F;

        Polyhedron_builder(Eigen::MatrixXd &V_input, Eigen::MatrixXi &F_input) : V(V_input), F(F_input) {}
    
        void operator() (HDS &hds);		
};

template<typename VectorType, typename IndexType>
void polyhedron_to_eigen(CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>, CGAL::Polyhedron_items_with_id_3 > &P,
        Eigen::PlainObjectBase<VectorType> &V, Eigen::PlainObjectBase<IndexType> &F);

#endif
