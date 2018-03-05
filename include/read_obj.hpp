#ifndef READ_OBJ_H
#define READ_OBJ_H

#include <Eigen/Dense>

#include <fstream>
#include <sstream>
#include <vector>
// definition for the function
bool loadOBJ (const std::string path, std::vector<double> &vertices, std::vector<double> &faces);

void print_vector(std::vector<double> &vector);

template<typename VectorType>
void read_row(std::istringstream &ss, std::vector<VectorType> &vector);

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
    }
    return 0;
}

namespace obj {
    int read(std::istream& input, std::vector<std::vector<double>> &V, std::vector<std::vector<int>> &F);
    int read(const std::string input_filename, std::vector<std::vector<double>> &V, std::vector<std::vector<int>> &F);
}
#endif
