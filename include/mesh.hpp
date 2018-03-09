#ifndef MESH_H
#define MESH_H

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

class MeshData {
    public:
        // constructor
        MeshData( void ) {}
        virtual ~MeshData( void ) {}

        // other constructors
        MeshData(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
        // get and set the data here
        
   // TODO make these private adn setup get/set methods. That way whenever either V,F or Polyhedron is modified the other is changed appropriately     
        // storing data of the mesh
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;

        CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>, CGAL::Polyhedron_items_with_id_3 > polyhedron;
    private:
        // build the polyhedron
        void build_polyhedron();
};

// TODO This declaration should remain private. Move ot only CPP file
// declaration for the polyhedron builder
template<typename HDS> 
class Polyhedron_builder : public CGAL::Modifier_base<HDS> {
    public:
        // TODO Think about making a giant class that holds P, V, F, and anything else important
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;

        Polyhedron_builder(const Eigen::MatrixXd &V_input, const Eigen::MatrixXi &F_input) : V(V_input), F(F_input) {}
    
        void operator() (HDS &hds);		
};
#endif
