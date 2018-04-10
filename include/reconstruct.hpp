#ifndef RECONSTRUCT_H
#define RECONSTRUCT_H

#include <Eigen/Dense>

class ReconstructMesh {
    public:
        ReconstructMesh( void ) {}
        virtual ~ReconstructMesh( void ) {}

        ReconstructMesh( const Eigen::Ref<const Eigen::MatrixXd> &v_in, 
                         const Eigen::Ref<const Eigen::MatrixXi> &f_in,
                         const Eigen::Ref<const Eigen::MatrixXd> &w_in );

        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        Eigen::MatrixXd weights;
};


#endif
