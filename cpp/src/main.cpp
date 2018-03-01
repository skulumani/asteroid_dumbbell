
#include "input_parser.hpp"
#include "read_obj.hpp"

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
    std::vector<std::vector<double>> V;
    std::vector<std::vector<int>> F;

    const std::string input_file = input.get_command_option("-i");
    if (!input_file.empty()) {
        std::cout << "Reading " << input_file << std::endl;
        std::ifstream input_stream(input_file);
        obj::read(input_stream, V, F);
        
    }  // input file is closed when leaving the scope
    
     
    return 0;
}
