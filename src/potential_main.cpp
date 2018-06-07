#include "potential.hpp"
#include "loader.hpp"
#include "mesh.hpp"

#include "input_parser.hpp"

#include <omp.h>
#include <iostream>

int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Potential test: potential_main -i obj_file.obj" << std::endl;
        return 0;
    }

    const std::string input_file = input.get_command_option("-i");
    if (input_file.empty()) {
        std::cout << "You need an input file" << std::endl;
        std::cout << "potential_main -i obj_file.obj" << std::endl;
        return 1;
    }

    std::shared_ptr<MeshData> mesh_data = Loader::load(input_file);
    
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
