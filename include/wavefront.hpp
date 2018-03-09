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


/**
    Read a single row of a OBJ file to a vector 

    @param ss Input string stream
    @returns vector Output vector to store the values
*/
template<typename VectorType>
void read_row(std::istringstream &ss, std::vector<VectorType> &vector);

namespace obj {

    class OBJ {
        public:
            // constructors
            OBJ() = default;
            OBJ(const std::string &input_filename);
            OBJ(const std::istream &input_stream);
            // TODO Check on copying/memory of eigen matrix initialization
            OBJ(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) : vertices(V), faces(F) {}
            
            void update(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);

        private:
            Eigen::MatrixXd vertices;
            Eigen::MatrixXi faces;
    };

}
#endif
