#ifndef LOADER_H
#define LOADER_H

#include <Eigen/Dense>
#include <memory>

// forward declaration
class MeshData;
using MeshPtr = std::shared_ptr<MeshData>;

class Loader {

    public:
        // factory methods to create a mesh
        static MeshPtr load(const std::string &input_filename);         
        static MeshPtr load(const std::istream &input_stream);
        static MeshPtr load(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
};
#endif
