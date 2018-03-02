#ifndef BUILD_POLY_H
#define BUILD_POLY_H

#include <CGAL/Polyhedron_3.h>

// declaration for the polyhedron builder
template<typename HDS> 
class Polyhedron_builder : public CGAL::Modifier_base<HDS> {
    public:
        Eigen::MatrixXd &V;
        Eigen::MatrixXi &F;

        Polyhedron_builder(Eigen::MatrixXd &V_input, Eigen::MatrixXi &F_input) : V(V_input), F(F_input) {};
		
		void operator()(HDS &hds);
};

#endif
