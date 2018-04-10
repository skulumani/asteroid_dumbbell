#ifndef RECONSTRUCT_H
#define RECONSTRUCT_H

#include "mesh.hpp"

#include <Eigen/Dense>
#include <memory>

class ReconstructMesh {
    public:
        ReconstructMesh( void ) {}
        virtual ~ReconstructMesh( void ) {}

        ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in, 
                         const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                         const Eigen::Ref<const Eigen::MatrixXd> &w_in );
        
        ReconstructMesh( std::shared_ptr<MeshData> mesh_in);

        // functions to access the private members
        Eigen::MatrixXd get_verts( void );
        Eigen::MatrixXi get_faces( void );
        Eigen::MatrixXd get_weights( void );

    private:
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        Eigen::MatrixXd weights;
};


#endif
