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
    int read(const std::string input_filename, std::vector<std::vector<double> > &V, 
            std::vector<std::vector<int> > &F);
    
    template <typename VectorType, typename IndexType> 
    int read_to_eigen(const std::string input_filename,
                        Eigen::PlainObjectBase<VectorType> &V,
                        Eigen::PlainObjectBase<IndexType> &F);

}

#endif
