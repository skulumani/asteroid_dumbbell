/**
    Read OBJ file to CGAL Mesh/Eigen arrays

    @author Shankar Kulumani
    @version 0.1 2/28/2018
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <assert.h>

#include "input_parser.hpp"

// Overload these functions eventually and save to a vector
// Single word read/print
// Read the fstream and save into an Eigen matrix (one for vertices and one for faces)

void print_vector(std::vector<double> &vector) {
    for (auto v = vector.begin(); v != vector.end(); ++v) {
        std::cout << " " << *v;
    }
    std::cout << std::endl;
}

template<typename VectorType> 
void read_row(std::istringstream &ss, std::vector<VectorType> &vector) {
    VectorType v;
    while (ss >> v) {
        vector.push_back(v);
    }
}

std::istream& read(std::istream& input, std::vector<std::vector<double>> &V, std::vector<std::vector<int>> &F) {

    // store some strings for parsing the obj file
    std::string v("v"); // vertices
    std::string f("f"); // faces
    std::string octothorp("#"); // comments
    
    std::string line;

    double vertex;  
    double vindex;

    while (std::getline(input, line)) {
        std::string row_type;
        std::istringstream row(line);

        row >> row_type;
        if (row_type == v) {
            std::vector<double> vertices;
            read_row(row, vertices);
            V.push_back(vertices);
            assert(vertices.size() == 3);
        } else if (row_type == f) {
            std::vector<int> indices;
            read_row(row, indices);
            F.push_back(indices);
            assert(indices.size() == 3);
        }
    }
    /* input.clear(); */
    return input;
}

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
        read(input_stream, V, F);
    }  // input file is closed when leaving the scope
        
    return 0;
}
