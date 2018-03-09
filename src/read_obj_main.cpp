
#include "input_parser.hpp"
#include "wavefront.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>


// TODO Add some tests
int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage read_obj -i input_file.obj" << std::endl;
    }
    
    // vectors of vectors to store the data
    std::vector<std::vector<double>> vector_V;
    std::vector<std::vector<int>> vector_F;
    int read_flag = 1;

    const std::string input_file = input.get_command_option("-i");
    if (!input_file.empty()) {
        std::cout << "Reading " << input_file << std::endl;
        obj::OBJ wavefront_obj(input_file);
        
        std::cout << "Vertices: \n" << wavefront_obj.vertices << std::endl;
        std::cout << "Faces: \n" << wavefront_obj.faces << std::endl;
    }  // input file is closed when leaving the scope

    return 0;
}
