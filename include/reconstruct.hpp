#ifndef RECONSTRUCT_H
#define RECONSTRUCT_H

#include "mesh.hpp"

#include <Eigen/Dense>
#include <memory>

class ReconstructMesh {
    public:
        ReconstructMesh( void ) {}
        virtual ~ReconstructMesh( void ) {}
    
        ReconstructMesh(const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                        const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                        const Eigen::Ref<const Eigen::MatrixXd> &w_in);

        ReconstructMesh( std::shared_ptr<MeshData> mesh_in);
        
        ReconstructMesh(const Eigen::Ref<const Eigen::MatrixXd> &v_in,
                        const Eigen::Ref<const Eigen::MatrixXi> &f_in);

        // Modify the vertices/weights with a new point
        void single_update(const Eigen::Ref<const Eigen::RowVector3d> &pt_in,
                        const double &max_angle);
        // now update given many points in an array
        void update(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& pts,
                const double& max_angle);

        // functions to access the private members
        Eigen::MatrixXd get_verts( void ) const;
        Eigen::MatrixXi get_faces( void ) const;
        Eigen::MatrixXd get_weights( void ) const;
        
        void update_meshdata( void );
    private:
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        Eigen::MatrixXd weights;

        std::shared_ptr<MeshData> mesh;
};

template<typename T>
Eigen::VectorXi vector_find(const Eigen::Ref<const T> &);

// compute the weights 
Eigen::Matrix<double, Eigen::Dynamic, 1> initial_weight(const Eigen::Ref<const Eigen::MatrixXd> &v_in);

#endif
