#include "potential.hpp"
#include "loader.hpp"
#include "mesh.hpp"

#include "input_parser.hpp"

#include <omp.h>
#include <iostream>

int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Potential test: \npotential_main -i obj_file.obj -n name" << std::endl;
        return 0;
    }

    const std::string input_file = input.get_command_option("-i");
    if (input_file.empty()) {
        std::cout << "You need an input file" << std::endl;
        std::cout << "potential_main -i obj_file.obj -n name" << std::endl;
        return 1;
    }
    
    const std::string name = input.get_command_option("-n");
    if (name.empty()) {
        std::cout << "You need the name: cube, castalia, itokawa" << std::endl;
        std::cout << "potential_main -i obj_file.obj -n name" << std::endl;
        return 1;
    }
    std::shared_ptr<MeshData> mesh_data = Loader::load(input_file);
    
    Asteroid ast(name, mesh_data);
    Eigen::Vector3d state; 
    state << 1, 2, 3;
    
    double start = omp_get_wtime();
    ast.polyhedron_potential(state);
    double end = omp_get_wtime() - start;
    std::cout << "Potential run time: " << end << " sec " << std::endl;

    std::cout << "U : " << ast.get_potential() << std::endl; 
    std::cout << "U_grad : " << ast.get_acceleration().transpose() << std::endl;
    std::cout << "U_grad_mat : \n" << ast.get_gradient_mat() << std::endl;
    std::cout << "U_laplace : " << ast.get_laplace() << std::endl;
    return 0;
}
