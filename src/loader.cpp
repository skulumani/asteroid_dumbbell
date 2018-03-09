// Mesh Loader factory methods to create the mesh class from some input data

#include "loader.hpp"
#include "mesh.hpp"

#include <Eigen/Dense>

MeshPtr load(const std::string &input_filename) {
    // read from filename and create eigen arrays
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    
    read_to_eigen(input_filename, vertices, faces);

    // instantiate the mesh and direct the pointer to it
    ptr = MeshPtr( new Mesh(vertices, faces) );

    return ptr;
}

