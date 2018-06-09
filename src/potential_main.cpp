#include "potential.hpp"
#include "loader.hpp"
#include "mesh.hpp"

#include "input_parser.hpp"

#include <omp.h>
#include <iostream>
#include <chrono>

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
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    ast.polyhedron_potential(state);
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();

    std::cout << "Potential time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " microsecond" << std::endl;
    std::cout << "Potential time: " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << " nanosecond " << std::endl;

    std::cout << "U : " << ast.get_potential() << std::endl; 
    std::cout << "U_grad : " << ast.get_acceleration().transpose() << std::endl;
    std::cout << "U_grad_mat : \n" << ast.get_gradient_mat() << std::endl;
    std::cout << "U_laplace : " << ast.get_laplace() << std::endl;

    /* // loop over the vertices */
    /* #pragma omp parallel */
    /* { */
    /*     #pragma omp single */
    /*     { */
    /*         #pragma omp task */
    /*         { */
    /*         for (Mesh::Vertex_range::iterator vb = mesh_data->vertices().begin(); */
    /*                 vb != mesh_data->vertices().end(); */
    /*                 ++vb) { */
    /*             std::cout << *vb << std::endl; */
    /*         } */
    /*         } */

    /*         #pragma omp task */
    /*         { */

    /*             for (Face_index fd : mesh_data->faces()) { */
    /*                 std::cout << fd << std::endl; */
    /*             } */
    /*         } */
    /*     } */
    /* } */
    return 0;
}
