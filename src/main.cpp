
#include "input_parser.hpp"
#include "read_obj.hpp"

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
        /* std::ifstream input_stream(input_file); */
        read_flag = obj::read(input_file, vector_V, vector_F);
        if (read_flag == 0) {
            std::cout << "Converting to Eigen arrays" << std::endl;
            Eigen::MatrixXd V;
            Eigen::MatrixXi F;
            vector_array_to_eigen(vector_V, V);
            vector_array_to_eigen(vector_F, F);
        }
         
    }  // input file is closed when leaving the scope

    return 0;
}
