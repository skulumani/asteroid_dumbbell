#include "potential.hpp"
#include "loader.hpp"
#include "mesh.hpp"
#include <omp.h>

#include <iostream>

int main() {
    
    std::shared_ptr<MeshData> mesh_data = Loader::load("./data/shape_model/CASTALIA/castalia.obj");

    std::shared_ptr<MeshParam> mesh_param = std::make_shared<MeshParam>(mesh_data);

    Asteroid ast("castalia", mesh_param);
    Eigen::Vector3d state; 
    state << 1, 2, 3;

    ast.polyhedron_potential(state);
    std::cout << "U : " << ast.get_potential() << std::endl; 
    std::cout << "U_grad : " << ast.get_acceleration().transpose() << std::endl;
    std::cout << "U_grad_mat : \n" << ast.get_gradient_mat() << std::endl;
    std::cout << "U_laplace : \n" << ast.get_laplace() << std::endl;
    return 0;
}
