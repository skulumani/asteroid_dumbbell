#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include <iostream>


// Start of polyhedron potential function code 
void face_contribution_loop(Eigen::Vector3d r_v,  Eigen::MatrixXd V, Eigen::MatrixXi F, 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> F_face, Eigen::Matrix<double, Eigen::Dynamic, 1> w_face) {
    
    std::cout << r_v << std::endl;
}

void laplacian_factor(Eigen::Array<double, Eigen::Dynamic, 3> r_v,
                     Eigen::Array<int, Eigen::Dynamic, 1> Fa,
                     Eigen::Array<int, Eigen::Dynamic, 1> Fb,
                     Eigen::Array<int, Eigen::Dynamic, 1> Fc) {
    // form the ri, rj, rk arrays
    Eigen::Array<double, Eigen::Dynamic, 3> ri, rj, rk;
    Eigen::Array<double, 1, 3> ra, rb, rc;

    for (int ii = 0; ii < Fa.rows(); ++ii) {
        ra = r_v.row(Fa(ii));
        rb = r_v.row(Fb(ii));
        rc = r_v.row(Fc(ii));

        ri.row(ii) << ra(0), ra(1), ra(2);
        /* rj.row(ii) = rb; */
        /* rk.row(ii) = rc; */

    }
    std::cout << ri.row(0) << std::endl;
    Eigen::Array<double, Eigen::Dynamic, 3> ri_norm, rj_norm, rk_norm;
    
    /* ri_norm = ri.pow(2); */
    /* std::cout << ri << std::endl; */
}

PYBIND11_MODULE(polyhedron_potential, m) {
    m.def("face_contribution_loop", &face_contribution_loop);
    m.def("laplacian_factor", &laplacian_factor);
}


