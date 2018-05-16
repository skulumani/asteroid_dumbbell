#include "potential.hpp"
#include "loader.hpp"
#include "mesh.hpp"
#include <omp.h>

#include <iostream>

int main() {
    
    std::shared_ptr<MeshData> mesh_data = Loader::load("./data/shape_model/CASTALIA/castalia.obj");
    
    double start = omp_get_wtime();
    std::shared_ptr<MeshParam> mesh_param = std::make_shared<MeshParam>(mesh_data);
    double end = omp_get_wtime() - start;
    std::cout << "MeshParam: " << end << " sec" << std::endl;

    Asteroid ast("castalia", mesh_param);
    Eigen::Vector3d state; 
    state << 1, 2, 3;
    
    start = omp_get_wtime();
    ast.polyhedron_potential(state);
    end = omp_get_wtime() - start;
    std::cout << "PolyPotential: " << end << " sec " << std::endl;

    std::cout << "U : " << ast.get_potential() << std::endl; 
    std::cout << "U_grad : " << ast.get_acceleration().transpose() << std::endl;
    std::cout << "U_grad_mat : \n" << ast.get_gradient_mat() << std::endl;
    std::cout << "U_laplace : \n" << ast.get_laplace() << std::endl;
    return 0;
}
