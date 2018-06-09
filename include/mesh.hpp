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
        MeshData(const Eigen::Ref<const Eigen::MatrixXd> &V,
                const Eigen::Ref<const Eigen::MatrixXi> &F);
        // get and set the data here
       
        /** @fn void update_mesh(const Eigen::Ref<const Eigen::MatrxiXd>& V, const Eigen::Ref<const Eigen::MatrixXi>& F)
                
            Completely update the mesh with new vertices and faces. This
            will also completely regenerate the mesh properties

            @param V Eigen matrix with the vertices
            @param F Eigen matrix with teh faces
            @returns void

            @author Shankar Kulumani
            @version 9 June 2018
        */ 
        void update_mesh(const Eigen::Ref<const Eigen::MatrixXd> &V, 
                const Eigen::Ref<const Eigen::MatrixXi> &F);

        // Surface mesh shit
        Mesh surface_mesh;
    
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

        std::size_t number_of_vertices( void ) const;
        std::size_t number_of_edges( void ) const;
        std::size_t number_of_faces( void ) const;
        std::size_t number_of_halfedges( void ) const;
        
        // Range types for iteration
        Mesh::Vertex_range vertices( void ) const;
        Mesh::Face_range faces( void ) const;
        Mesh::Edge_range edges( void ) const;
        Mesh::Halfedge_range halfedges( void ) const;
        
        /* std::shared_ptr<Mesh> get_mesh( void ) const { return std::make_shared<Mesh>(surface_mesh) }; */

        template<typename Index>
        Eigen::RowVector3d get_vertex(const Index& index) const;
        
        /** @fn bool set_vertex(const Vertex_index& vd, 
         *              const Eigen::Ref<const Eigen::Vector3d>& vec)
                
            Update the position of a single vertex and recompute the affected
            mesh properties

            @param vd Vertex index to modify
            @param vec Eigen vector of the new vertex position
            @returns bool True if good

            @author Shankar Kulumani
            @version 9 June 2018
        */
        bool set_vertex(const Vertex_index& vd,
                const Eigen::Ref<const Eigen::Vector3d>& vec);

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

        void build_surface_mesh(
                const Eigen::Ref<const Eigen::MatrixXd>& V,
                const Eigen::Ref<const Eigen::MatrixXi>& F);
        
        /** @fn bool build_face_properties( void )
                
            Compute the face properties for all faces
            This basically calls compute_face_properties for each face

            @param void 
            @returns bool true if sucess

            @author Shankar Kulumani
            @version Date
        */
        bool build_face_properties( void );
        /** @fn bool update_face_properties(const std::vector<Face_index>& face_vec)
                
            Update the face properties, normal, center, and dyad, of faces listed
            in the vector

            @param face_vec std::vector holding the faces to modify
            @returns bool true if successful

            @author Shankar Kulumani
            @version 9 June 2018
        */
        bool update_face_properties(const std::vector<Face_index>& face_vec) ;
        
        /** @fn bool compute_face_properties(const Face_index& fd)
                
            Actually compute the face properties for the given face

            @param fd Face_index of face to modify
            @returns bool true if successful

            @author Shankar Kulumani
            @version 9 June 2018
        */
        bool compute_face_properties(const Face_index& fd);

        bool build_halfedge_properties( void );
        bool build_edge_properties( void ); 

        /** @fn std::vector<Face_index> get_faces_with_vertex(const Vertex_index& vd)
                
            Get all face indices which contain this vertex vd

            @param vd Vertex index of the vertex in question
            @returns fd_vec Face index std::vector of all faces

            @author Shankar Kulumani
            @version 9 June 2018
        */
        std::vector<Face_index> get_faces_with_vertex(const Vertex_index& vd) const;
        
        /** @fn std::vector<Halfedge_index> get_halfedges_with_vertex(
         *                      const Vertex_index& vd) const
                
            Get all the halfedges with this vertex as a source or target

            @param vd Vertex index
            @returns halfedge_vec std:;vector of all halfedges associated with 
            this vertex

            @author Shankar Kulumani
            @version 9 June 2018
        */
        std::vector<Halfedge_index> get_halfedges_with_vertex(const Vertex_index& vd) const;
        
};


#endif
