#ifndef READ_OBJ_H
#define READ_OBJ_H

#include <fstream>
#include <sstream>
#include <vector>
// definition for the function
bool loadOBJ (const std::string path, std::vector<double> &vertices, std::vector<double> &faces);

void print_vector(std::vector<double> &vector);

template<typename VectorType>
void read_row(std::istringstream &ss, std::vector<VectorType> &vector);

namespace obj {
    int read(std::istream& input, std::vector<std::vector<double>> &V, std::vector<std::vector<int>> &F);
    int read(const std::string input_filename, std::vector<std::vector<double>> &V, std::vector<std::vector<int>> &F);
}
#endif
