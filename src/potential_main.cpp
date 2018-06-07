#include "potential.hpp"
#include "loader.hpp"
#include "mesh.hpp"
#include <omp.h>

#include <iostream>

int main() {
    
    std::shared_ptr<MeshData> mesh_data = Loader::load("./integration/cube.obj");
    
    Asteroid ast("cube", mesh_data);
    Eigen::Vector3d state; 
    state << 1, 2, 3;
    
    /* /1* start = omp_get_wtime(); *1/ */
    ast.polyhedron_potential(state);
    /* /1* end = omp_get_wtime() - start; *1/ */
    /* /1* std::cout << "PolyPotential: " << end << " sec " << std::endl; *1/ */

    std::cout << "U : " << ast.get_potential() << std::endl; 
    std::cout << "U_grad : " << ast.get_acceleration().transpose() << std::endl;
    std::cout << "U_grad_mat : \n" << ast.get_gradient_mat() << std::endl;
    std::cout << "U_laplace : " << ast.get_laplace() << std::endl;
    return 0;
}
