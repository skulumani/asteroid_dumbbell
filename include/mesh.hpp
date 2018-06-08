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
       
        
        // update mesh from arrays and faces array
        void update_mesh(const Eigen::Ref<const Eigen::MatrixXd> &V, 
                const Eigen::Ref<const Eigen::MatrixXi> &F);

        Polyhedron polyhedron;
       
        // Surface mesh shit
        Mesh surface_mesh;
    
        // indices to interface with the surface_mesh
        std::vector<Vertex_index> vertex_descriptor;
        std::vector<std::vector<Vertex_index> > vertex_in_face_descriptor;
        std::vector<Face_index> face_descriptor;
        
        // L Edge factor (function of current position)
        bool build_edge_factor(const Eigen::Ref<const Eigen::Vector3d>& pos);
        // w face factor (function of the current position)
        bool build_face_factor(const Eigen::Ref<const Eigen::Vector3d>& pos);

        // GETTERS
        // get a specific face/vertex using an index like access operator
        //
        // convert surface mesh to eigen arrays
        Eigen::Matrix<double, Eigen::Dynamic, 3> get_verts( void ) const;
        Eigen::Matrix<int, Eigen::Dynamic, 3> get_faces( void ) const;

        double number_of_vertices( void ) const;
        double number_of_edges( void ) const;
        double number_of_faces( void ) const;
        double number_of_halfedges( void ) const;
        
        // Range types for iteration
        Mesh::Vertex_range vertices( void ) const;
        Mesh::Face_range faces( void ) const;
        Mesh::Edge_range edges( void ) const;
        Mesh::Halfedge_range halfedges( void ) const;

        template<typename Index>
        Eigen::RowVector3d get_vertex(const Index& index) const;

        template<typename Index>
        Eigen::RowVector3i get_face_vertices(const Index& index) const;
        
        // Getters for the property maps
        template<typename Index>
        Eigen::Vector3d get_face_normal(const Index& fd) const;
        template<typename Index>
        Eigen::Vector3d get_face_center(const Index& fd) const;
        template<typename Index>
        Eigen::Matrix3d get_face_dyad(const Index& fd) const;
        
        template<typename Index>
        Eigen::Vector3d get_halfedge_normal(const Index& hd) const;
        
        template<typename Index>
        Eigen::Matrix3d get_edge_dyad(const Index& ed) const;
        
        template<typename Index>
        double get_edge_factor(const Index& ed) const;

        template<typename Index>
        double get_face_factor(const Index& ed) const;
        double get_sum_face_factor( void ) const;
    private:

        // build the polyhedron
        void build_polyhedron(
                const Eigen::Ref<const Eigen::MatrixXd>& V,
                const Eigen::Ref<const Eigen::MatrixXi>& F);
        void build_surface_mesh(
                const Eigen::Ref<const Eigen::MatrixXd>& V,
                const Eigen::Ref<const Eigen::MatrixXi>& F);

        // compute normal to each face of surface_mesh
        bool build_face_properties( void );
        bool build_halfedge_properties( void );
        bool build_edge_properties( void ); 
};


#endif
