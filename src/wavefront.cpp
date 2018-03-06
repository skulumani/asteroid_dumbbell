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
    
    
    template<typename VectorType, typename IndexType> 
    int read(const std::string input_filename, Eigen::PlainObjectBase<VectorType> &V, Eigen::PlainObjectBase<IndexType> &F) {
        // just call the stl vector version
        std::vector<std::vector<double> > V_vector;
        std::vector<std::vector<int> > F_vector;
        int read_flag = obj::read(input_filename, V_vector, F_vector);
        V.resize(V_vector.size(), 3);
        F.resize(F_vector.size(), 3);

        if (read_flag == 0) {
            obj::vector_array_to_eigen(V_vector,  V);
            obj::vector_array_to_eigen(F_vector, F);
            return 0;
        } else {
            V = Eigen::MatrixX3d::Zero();
            F = Eigen::MatrixX3i::Zero();
            return 1;
        }

    }

    void print_vector(std::vector<double> &vector) {
        for (auto v = vector.begin(); v != vector.end(); ++v) {
            std::cout << " " << *v;
        }
        std::cout << std::endl;
    }
        
} // namespace read_obj
