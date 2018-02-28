/**
    Read OBJ file to CGAL Mesh/Eigen arrays

    @author Shankar Kulumani
    @version 0.1 2/28/2018
*/
#include <iostream>
#include <fstream>
#include <vector>

#include "input_parser.hpp"

// Overload these functions eventually and save to a vector
// Single word read/print
// Read the fstream and save into an Eigen matrix (one for vertices and one for faces)
std::istream& read(std::istream& input) {
    if (input) {
        std::string word;
        while (input >> word) {
            std::cout << "input is : " << word << std::endl;
        }
    } else {
        std::cout << "Error opening the file" << std::endl;
    }

    /* input.clear(); */
    return input;
}


int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage read_obj -i input_file.obj" << std::endl;
    }

    const std::string input_file = input.get_command_option("-i");
    if (!input_file.empty()) {
        std::cout << "Reading " << input_file << std::endl;
        std::ifstream input_stream(input_file);
        read(input_stream);
    } // input file goes out of scope and is closed
     
    return 0;
}
