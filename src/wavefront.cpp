/**
    Read OBJ file to CGAL Mesh/Eigen arrays

    @author Shankar Kulumani
    @version 0.1 2/28/2018
*/

#include "wavefront.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <assert.h>


template<typename VectorType> 
void read_row(std::istringstream &ss, std::vector<VectorType> &vector) {
    VectorType v;
    while (ss >> v) {
        vector.push_back(v);
    }
}

namespace obj {


    int read(std::istream& input, std::vector<std::vector<double>> &V, std::vector<std::vector<int>> &F) {
        
        if (input.fail()) {
            std::cout << "Error opening the file stream" << std::endl;
            return 1;
        }
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
				int v;
				while (row >> v) {
					indices.push_back(v - 1);
				}
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
        if (!input_stream.fail()) {
            int read_flag = obj::read(input_stream, V, F);
            return 0;
        } else {
            std::cout << "Error opening file filename" << std::endl;
            return 1;
        }
    }

    void print_vector(std::vector<double> &vector) {
        for (auto v = vector.begin(); v != vector.end(); ++v) {
            std::cout << " " << *v;
        }
        std::cout << std::endl;
    }
        
template<typename VectorType, typename Derived> 
int vector_array_to_eigen(std::vector<std::vector<VectorType> > &vector,
        Eigen::PlainObjectBase<Derived> &matrix) {
    // initialize a matrix to hold everything (assumes all are the same size
    int rows = vector.size();
    int cols = vector[0].size();
    matrix.resize(rows, cols);
    for (int ii = 0; ii < rows; ii++) {
        Eigen::Matrix<typename Derived::Scalar, 1, 3> v(vector[ii].data());
        matrix.row(ii) = v;
    }
    return 0;
}
} // namespace read_obj

// Explicit initialization


template int obj::vector_array_to_eigen<double, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(std::vector<std::vector<double, std::allocator<double> >, std::allocator<std::vector<double, std::allocator<double> > > >&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);

template int obj::vector_array_to_eigen<int, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);
