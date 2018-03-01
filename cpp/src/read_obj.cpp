/**
    Read OBJ file to CGAL Mesh/Eigen arrays

    @author Shankar Kulumani
    @version 0.1 2/28/2018
*/

#include "input_parser.hpp"
#include "read_obj.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <assert.h>

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

namespace obj {


    int read(std::istream& input, std::vector<std::vector<double>> &V, std::vector<std::vector<int>> &F) {

        // store some strings for parsing the obj file
        std::string v("v"); // vertices
        std::string f("f"); // faces
        std::string octothorp("#"); // comments

        std::string line;

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
        return 0;
    }
    // overloaded function for opening the file
    int read(const std::string input_filename, std::vector<std::vector<double>> &V, std::vector<std::vector<int>> &F) {
        std::ifstream input_stream;
        input_stream.open(input_filename);

        // check to make sure the file is opened properly
        if (input_stream.is_open()) {
            obj::read(input_stream, V, F);
        } else {
            std::cout << "Error opening file" << std::endl;
            return 1;
        }
        return 0;
    }
} // namespace read_obj
