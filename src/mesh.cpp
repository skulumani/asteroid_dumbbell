#include "mesh.hpp"

#include <Eigen/Dense>

MeshData::MeshData(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
    this->vertices = V;
    this->faces = F;
}
