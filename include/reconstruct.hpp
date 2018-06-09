#ifndef RECONSTRUCT_H
#define RECONSTRUCT_H

#include "mesh.hpp"

#include <Eigen/Dense>
#include <memory>
/** @class ReconstructMesh

    @brief Mesh reconstruction using radial vertex adjustment for an asteroid
    
    Given datapoints/measurements of the surface of an asteroid, in the asteroid 
    fixed frame. This will update the mesh, and an uncertainty metric, for each
    vertex by radially changing its position

    @author Shankar Kulumani
    @version 31 May 2018
*/
class ReconstructMesh {
    public:
        ReconstructMesh( void ) {}
        virtual ~ReconstructMesh( void ) {}
        
        ReconstructMesh(const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                        const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                        const Eigen::Ref<const Eigen::VectorXd> &w_in);

        ReconstructMesh( std::shared_ptr<MeshData> mesh_in);
        
        ReconstructMesh(const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                        const Eigen::Ref<const Eigen::MatrixXi> &f_in);
        
        /** @fn void single_update(const Eigen::Ref<const Eigen::RowVector3d>& pt_in,
         *                         const double & max_angle)
                
            Update mesh by taking a single measurement, in the asteroid fixed frame,
            and moving the vertices that are close to it (angular seperation)

            @param pt_in Row vector of measurement in the asteroid frame
            @param max_angle Max angular seperation between the measurement and 
                associated vertices
            @returns None

            @author Shankar Kulumani
            @version 31 May 2018
        */
        void single_update(const Eigen::Ref<const Eigen::RowVector3d> &pt_in,
                        const double &max_angle);
        /** @fn void update(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& pts,
         *                  const double& max_angle)
                
            Incorporate many points by treating each one in sequence using 
            single_update

            @param pots Array of measurements, nx3, in the asteroid frame
            @param max_angle Max angular seperation between the measurement and 
                associated vertices
            @returns None

            @author Shankar Kulumani
            @version 31 May 2018
        */
        void update(
                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& pts,
                const double& max_angle);

        // functions to access the private members
        Eigen::MatrixXd get_verts( void ) const;
        Eigen::MatrixXi get_faces( void ) const;
        Eigen::VectorXd get_weights( void ) const;
        
        std::size_t number_of_vertices( void ) const { return mesh->number_of_vertices(); }
        std::size_t number_of_faces( void ) const { return mesh->number_of_faces(); }

        std::shared_ptr<MeshData> get_mesh( void ) const { return mesh; }
        
        void update_meshdata( void );

    private:
        
        double maximum_weight(const Eigen::Ref<const Eigen::Vector3d>& v_in);
        bool initialize_weight( void );
        bool set_all_weights( const Eigen::Ref<const Eigen::VectorXd>& w_in );
        
        bool set_weight(const Vertex_index& vd, const double& w);
        double get_weight(const Vertex_index& vd);

        Eigen::VectorXd weights; /**< Weight for each vertex */

        std::shared_ptr<MeshData> mesh; /**< MeshData holding vertices */
};

/* template<typename T> */
/* Eigen::VectorXi vector_find(const Eigen::Ref<const T> &); */

// compute the weights 
Eigen::Matrix<double, Eigen::Dynamic, 1> initial_weight(const Eigen::Ref<const Eigen::MatrixXd> &v_in);

#endif
