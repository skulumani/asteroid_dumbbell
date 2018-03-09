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


    /**
        Read OBJ file to vector of vectors

        @param input string stream for input OBJ file
        @returns V vector of vector doubles for vertices
        @return F vector of vector ints for faces
    */
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
    /**
        Read OBJ file to vector of vectors

        @param input_filename string of the file. This just opens the file
        @returns V vector of vector doubles for vertices
        @return F vector of vector ints for faces
    */
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

    /**
        Print a row from a stl vectors

        @param vector Vector input to print 
    */
    void print_vector(std::vector<double> &vector) {
        for (auto v = vector.begin(); v != vector.end(); ++v) {
            std::cout << " " << *v;
        }
        std::cout << std::endl;
    }
        
    /**
      Convert vector of vectors to Eigen arrays

      @param vector Vector of vectors 
      @returns matrix Output eigen matrix 
      */
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

    template <typename VectorType, typename IndexType> 
    int read_to_eigen(const std::string input_filename, Eigen::PlainObjectBase<VectorType> &V,
            Eigen::PlainObjectBase<IndexType> &F) {
        // just call the stl vector version
        std::vector<std::vector<double> > V_vector;
        std::vector<std::vector<int> > F_vector;
        int read_flag = obj::read(input_filename, V_vector, F_vector);
        V.resize(V_vector.size(), 3);
        F.resize(F_vector.size(), 3);

        if (read_flag == 0) {
            vector_array_to_eigen(V_vector,  V);
            vector_array_to_eigen(F_vector, F);
            return 0;
        } else {
            V = Eigen::MatrixXd::Zero(V_vector.size(), 3);
            F = Eigen::MatrixXi::Zero(F_vector.size(), 3);
            return 1;
        }

    }
    /***********************************OBJ CLASS*************************** */
    OBJ::OBJ(const std::string &input_filename) {
        read_to_eigen(input_filename, this->vertices, this->faces);
    }

    OBJ::OBJ(const std::istream &input_stream) {
        std::vector<std::vector<double>> V;
        std::vector<std::vector<int>> F;
        read(input_stream, V, F);
        // store to object
        vector_array_to_eigen(V, this->vertices);
        vector_array_to_eigen(F, this->faces);
    }

    void OBJ::update(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
        this->vertices = V;
        this->faces = F;
    }
} // namespace read_obj

// Explicit initialization
template int obj::vector_array_to_eigen<double, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(std::vector<std::vector<double, std::allocator<double> >, std::allocator<std::vector<double, std::allocator<double> > > >&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);

template int obj::vector_array_to_eigen<int, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);

template int obj::read_to_eigen<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);
