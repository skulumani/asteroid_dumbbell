/**
    Read OBJ file to CGAL Mesh/Eigen arrays

    @author Shankar Kulumani
    @version 0.1 2/28/2018
*/
#include <iostream>
#include <fstream>
#include "input_parser.hpp"

std::istream& read_cin(std::istream& input) {
    std::string word;
    while (input >> word) {
        std::cout << "input is : " << word << std::endl;
    }
    /* input.clear(); */
    return input;
}

int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage read_obj -i input_file.obj" << std::endl;
    }

    const std::string &input_file = input.get_command_option("-i");
    if (!input_file.empty()) {
        std::cout << "Filename = " << input_file << std::endl;
        std::istream &input_stream = read_cin(std::cin) ;

        std::cout << input_stream.good() << std::endl;
        /* std::string word; */
        /* while (std::cin >> word) { */
        /*     std::cout << "input is : " << word << std::endl; */
        /* } */
        /* std::cout << std::endl; */
        // open the file and check that it is open
        
        // read a line from it and print to the screen

        /* std::string line; */    
        /* while (std::getline(ifile, line)) { */
        /*     std::cout << line << std::endl; */
        /* } */
    }
    /* std::cout << "path is " << ipath << std::endl; */
    /* std::cout << argv[0] << argv[1] << std::endl; */
    /* std::cout << argc << std::endl; */
    /* std::string pathname = "cube.obj"; */

    /* std::ifstream ifile(pathname); */

    /* if (!ifile.is_open()) { */
    /*     std::cout << "Could not open " << pathname << std::endl; */
    /*     return 1; */
    /* } */

    
    return 0;
}
