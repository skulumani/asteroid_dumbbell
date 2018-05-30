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
        MeshData(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXi> &F);
        // get and set the data here
       
        // member functions
        Eigen::MatrixXd get_verts( void ) const { return vertices; }
        Eigen::MatrixXi get_faces( void ) const { return faces; }
        
        // update mesh from arrays and faces array
        void update_mesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);

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


#endif
