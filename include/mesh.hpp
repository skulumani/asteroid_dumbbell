#ifndef MESH_H
#define MESH_H

#include "cgal_types.hpp"

#include <Eigen/Dense>

#include <vector>

// This data holds the polyhedorn and mesh
class MeshData {
    public:
        // constructor
        MeshData( void ) {}
        virtual ~MeshData( void ) {}

        // other constructors
        MeshData(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
        // get and set the data here
        
        // storing data of the mesh
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;

        Polyhedron polyhedron;
       
        // Surface mesh shit
        Mesh surface_mesh;
        
        std::vector<Vertex_index> vertex_descriptor;
        std::vector<std::vector<Vertex_index> > face_descriptor;

    private:
        // build the polyhedron
        void build_polyhedron();
        void build_surface_mesh();
};

// declaration for the polyhedron builder
template<typename HDS> 
class Polyhedron_builder : public CGAL::Modifier_base<HDS> {
    public:
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;

        Polyhedron_builder(const Eigen::MatrixXd &V_input, const Eigen::MatrixXi &F_input) : V(V_input), F(F_input) {}
    
        void operator() (HDS &hds);		
};

#endif
