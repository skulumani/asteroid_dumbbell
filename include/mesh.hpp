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
    
        // indices to interface with the surface_mesh
        std::vector<Vertex_index> vertex_descriptor;
        std::vector<std::vector<Vertex_index> > vertex_in_face_descriptor;
        std::vector<Face_index> face_descriptor;
        
        // convert surface mesh to eigen arrays
        Eigen::Matrix<double, Eigen::Dynamic, 3> get_surface_mesh_vertices( void );
        Eigen::Matrix<int, Eigen::Dynamic, 3> get_surface_mesh_faces( void );

        // get a specific face/vertex using an index like access operator
        template<typename Index>
        Eigen::RowVector3d get_vertex(const Index& index);

        template<typename Index>
        Eigen::RowVector3i get_face_vertices(const Index& index);

        // compute normal to each face of surface_mesh
        void build_face_normals( void );
    private:

        // build the polyhedron
        void build_polyhedron();
        void build_surface_mesh();
        
};


#endif
