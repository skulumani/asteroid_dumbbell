#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include <iostream>


// Start of polyhedron potential function code 
void face_contribution_loop(Eigen::Vector3d r_v,  Eigen::MatrixXd V, Eigen::MatrixXi F, 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> F_face, Eigen::Matrix<double, Eigen::Dynamic, 1> w_face) {
    
    std::cout << r_v << std::endl;
}

void laplacian_factor(Eigen::Matrix<double, Eigen::Dynamic, 3> r_v,
                     Eigen::Matrix<int, Eigen::Dynamic, 1> Fa,
                     Eigen::Matrix<int, Eigen::Dynamic, 1> Fb,
                     Eigen::Matrix<int, Eigen::Dynamic, 1> Fc) {
    // form the ri, rj, rk arrays
    Eigen::Matrix<int, Eigen::Dynamic, 3> ri, rj, rk;

    for (int ii = 0; ii < Fa.rows(); ++ii) {
        std::cout << r_v.row(ii) << std::endl;
    }
}

PYBIND11_MODULE(polyhedron_potential, m) {
    m.def("face_contribution_loop", &face_contribution_loop);
    m.def("laplacian_factor", &laplacian_factor);
}


