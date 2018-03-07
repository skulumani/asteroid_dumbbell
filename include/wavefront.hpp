/**
    Read OBJ file into a std::vector or Eigen array

    @author Shankar Kulumani
    @version 5 March 2018
*/
#ifndef READ_OBJ_H
#define READ_OBJ_H


#include <Eigen/Dense>

#include <fstream>
#include <sstream>
#include <vector>

// definition for the function
bool loadOBJ (const std::string path, std::vector<double> &vertices, std::vector<double> &faces);


/**
    Read a single row of a OBJ file to a vector 

    @param ss Input string stream
    @returns vector Output vector to store the values
*/
template<typename VectorType>
void read_row(std::istringstream &ss, std::vector<VectorType> &vector);


namespace obj {
    /**
        Print a row from a stl vectors

        @param vector Vector input to print 
    */
    void print_vector(std::vector<double> &vector);

    /**
      Convert vector of vectors to Eigen arrays

      @param vector Vector of vectors 
      @returns matrix Output eigen matrix 
      */
    template<typename VectorType, typename Derived> 
        int vector_array_to_eigen(std::vector<std::vector<VectorType> > &vector,
                Eigen::PlainObjectBase<Derived> &matrix);
    /**
        Read OBJ file to vector of vectors

        @param input string stream for input OBJ file
        @returns V vector of vector doubles for vertices
        @return F vector of vector ints for faces
    */
    int read(std::istream& input, std::vector<std::vector<double> > &V,
            std::vector<std::vector<int> > &F);
    /**
        Read OBJ file to vector of vectors

        @param input_filename string of the file. This just opens the file
        @returns V vector of vector doubles for vertices
        @return F vector of vector ints for faces
    */
    int read(const std::string input_filename, std::vector<std::vector<double>
            > &V, std::vector<std::vector<int> > &F);
    
    template <typename VectorType, typename IndexType> int read_to_eigen(const
            std::string input_filename, Eigen::PlainObjectBase<VectorType> &V,
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

}

#endif
