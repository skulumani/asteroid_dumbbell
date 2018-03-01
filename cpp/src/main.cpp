
#include "input_parser.hpp"
#include "read_obj.hpp"

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>

//function to convert vector<vector>> to MatrixXd
template<typename VectorType, typename Derived> 
int vector_array_to_eigen(std::vector<std::vector<VectorType>> &vector,
        Eigen::PlainObjectBase<Derived> &matrix) {
    // initialize a matrix to hold everything (assumes all are the same size
    int rows = vector.size();
    int cols = vector[0].size();
    matrix.resize(rows, cols);
    for (int ii = 0; ii < rows; ii++) {
        Eigen::Matrix<typename Derived::Scalar, 1, 3> v(vector[ii].data());
        matrix.row(ii) = v;
        /* std::cout << vector[ii] << std::endl; */
        /* for (int jj = 0; jj < cols; jj++) { */
        /*     matrix(ii, jj) = vector[ii][jj]; */
        /* } */
    }
    return 0;
}

// TODO Add some tests
int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage read_obj -i input_file.obj" << std::endl;
    }
    
    // vectors of vectors to store the data
    std::vector<std::vector<double>> vector_V;
    std::vector<std::vector<int>> vector_F;

    const std::string input_file = input.get_command_option("-i");
    if (!input_file.empty()) {
        std::cout << "Reading " << input_file << std::endl;
        std::ifstream input_stream(input_file);
        obj::read(input_stream, vector_V, vector_F);
        
    }  // input file is closed when leaving the scope
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    vector_array_to_eigen(vector_V, V);
    vector_array_to_eigen(vector_F, F);
    std::cout << V << std::endl;
    return 0;
}
