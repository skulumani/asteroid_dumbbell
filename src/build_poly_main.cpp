#include "input_parser.hpp"
#include "wavefront.hpp"
#include "polyhedron.hpp"

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <iostream>
#include <fstream>
#include <vector>

typedef CGAL::Simple_cartesian<double>     Kernel;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>         Polyhedron;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;


int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage read_obj -i input_file.obj" << std::endl;
    }
    
    // vectors of vectors to store the data
    std::vector<std::vector<double>> vector_V;
    std::vector<std::vector<int>> vector_F;
    int read_flag = 1;
    Polyhedron P;

    const std::string input_file = input.get_command_option("-i");
    if (!input_file.empty()) {
        std::cout << "Reading " << input_file << std::endl;
        obj::OBJ cube(input_file);

        std::cout << "Now initializing a Poly object to store all of our data in a single structure/object" << std::endl;

        Mesh P_eigen(cube.vertices, cube.faces);
        Mesh P_string(input_file);
         
        // Now we'll extract the vertices adn faces
        /* auto eigen_arrays = P_eigen.get_arrays(); */
        
        /* std::cout << "Vertices from P: \n" << eigen_arrays.vertices << std::endl; */

    }  // input file is closed when leaving the scope
    /* Eigen::MatrixXd V_poly; */
    /* Eigen::MatrixXi F_poly; */
    std::cout << "Can extract V, F from a polyhedron now" << std::endl;

    /* polyhedron_to_eigen(P, V_poly, F_poly); */
    
    return 0;
}
