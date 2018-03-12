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
    Eigen::Matrix<double, Eigen::Dynamic, 3> ri, rj, rk, rjrk_cross;
    Eigen::Matrix<double, Eigen::Dynamic, 1> ri_norm, rj_norm, rk_norm;

    ri.resize(Fa.rows(), 3);
    rj.resize(Fb.rows(), 3);
    rk.resize(Fc.rows(), 3);
    rjrk_cross.resize(Fa.rows(), 3);
    
    ri_norm.resize(Fa.rows(), 1);
    rj_norm.resize(Fb.rows(), 1);
    rk_norm.resize(Fc.rows(), 1);

    Eigen::Matrix<double, 1, 3> ra, rb, rc;

    for (int ii = 0; ii < Fa.rows(); ++ii) {
        ra = r_v.row(Fa(ii));
        rb = r_v.row(Fb(ii));
        rc = r_v.row(Fc(ii));

        ri.row(ii) = ra;
        rj.row(ii) = rb;
        rk.row(ii) = rc;

        rjrk_cross.row(ii) = rj.row(ii).cross(rk.row(ii));

    }

   
    ri_norm = ri.array().pow(2).rowwise().sum().sqrt();
    rj_norm = rj.array().pow(2).rowwise().sum().sqrt();
    rk_norm = rk.array().pow(2).rowwise().sum().sqrt();
    // dot product terms

    // numerator and denonminator of atan2
    Eigen::Matrix<double, Eigen::Dynamic, 1> num, den;
    num = (ri.array() * rjrk_cross.array()).rowwise().sum();
    
    std::cout << num << std::endl;
}

PYBIND11_MODULE(polyhedron_potential, m) {
    m.def("face_contribution_loop", &face_contribution_loop);
    m.def("laplacian_factor", &laplacian_factor);
}


