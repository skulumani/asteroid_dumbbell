#include "potential.hpp"

#include <iostream>

void polyhedron_parameters(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> > & V,
        const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 3> >& F) {
    
    std::size_t num_v = V.rows();
    std::size_t num_f = F.rows();
    std::size_t num_e = 3 * (num_v -2);

    // calculate all the edges zero based. This assumes it's already subtracted from the original OBJ for example
    Eigen::Array<int, Eigen::Dynamic, 1> Fa, Fb, Fc;
    Fa = F.col(0);
    Fb = F.col(1);
    Fc = F.col(2);

    Eigen::Array<double, Eigen::Dynamic, 3> V1, V2, V3;
    V1.resize(Fa.rows(),3 );
    V2.resize(Fb.rows(),3 );
    V3.resize(Fc.rows(),3 );
    for (int ii = 0; ii < num_f; ++ii) {
        V1.row(ii) << V(Fa(ii), 0), V(Fa(ii), 1), V(Fa(ii), 2);
        V2.row(ii) << V(Fb(ii), 0), V(Fb(ii), 1), V(Fb(ii), 2);
        V3.row(ii) << V(Fc(ii), 0), V(Fb(ii), 1), V(Fc(ii), 2);
    }

    // compute the edge vectors
    Eigen::Array<double, Eigen::Dynamic, 3> e1, e2, e3;
    e1 = V2 - V1;
    e2 = V3 - V2;
    e3 = V1 - V3;
    
    // vertex map
    Eigen::Array<int, Eigen::Dynamic, 2> e1_vertex_map, e2_vertex_map, e3_vertex_map, e_vertex_map;
    e1_vertex_map.resize(Fa.rows(), 2);
    e2_vertex_map.resize(Fb.rows(), 2);
    e3_vertex_map.resize(Fc.rows(), 2);
    e_vertex_map.resize(3*Fc.rows(), 2); 

    e1_vertex_map.col(0) = Fb;
    e1_vertex_map.col(1) = Fa;

    e2_vertex_map.col(0) = Fc;
    e2_vertex_map.col(1) = Fb;

    e3_vertex_map.col(0) = Fa;
    e3_vertex_map.col(1) = Fc;
    
    e_vertex_map << e1_vertex_map, e2_vertex_map, e3_vertex_map;
    std::cout << std::sort(e_vertex_map << std::endl;
}

// Start of polyhedron potential function code 
void face_contribution_loop(Eigen::Vector3d r_v,  Eigen::MatrixXd V, Eigen::MatrixXi F, 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> F_face, Eigen::Matrix<double, Eigen::Dynamic, 1> w_face) {
    
    std::cout << r_v << std::endl;
}

int laplacian_factor(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& r_v,
                     const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 1> >& Fa,
                     const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 1> >& Fb,
                     const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 1> >& Fc,
                     Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > w_face) {
    // form the ri, rj, rk arrays
    Eigen::Array<double, Eigen::Dynamic, 3> ri, rj, rk, rjrk_cross;

    ri.resize(Fa.rows(), 3);
    rj.resize(Fb.rows(), 3);
    rk.resize(Fc.rows(), 3);
    rjrk_cross.resize(Fa.rows(), 3);
    
    Eigen::Array<double, 1, 3> ra, rb, rc;

    for (int ii = 0; ii < Fa.rows(); ++ii) {
        ra = r_v.row(Fa(ii));
        rb = r_v.row(Fb(ii));
        rc = r_v.row(Fc(ii));

        ri.row(ii) = ra;
        rj.row(ii) = rb;
        rk.row(ii) = rc;

        rjrk_cross.row(ii) = rj.row(ii).matrix().cross(rk.row(ii).matrix());

    }

    Eigen::Array<double, Eigen::Dynamic, 1> ri_norm, rj_norm, rk_norm;
    ri_norm = ri.pow(2).rowwise().sum().sqrt();
    rj_norm = rj.pow(2).rowwise().sum().sqrt();
    rk_norm = rk.pow(2).rowwise().sum().sqrt();

    // dot product terms
    Eigen::Array<double, Eigen::Dynamic, 1> rjrk_dot, rkri_dot, rirj_dot;
    rjrk_dot = (rj * rk).rowwise().sum();
    rkri_dot = (rk * ri).rowwise().sum();
    rirj_dot = (ri * rj).rowwise().sum();

    // numerator and denonminator of atan2
    Eigen::Array<double, Eigen::Dynamic, 1> num, den;
    num = (ri * rjrk_cross).rowwise().sum();
    den = ri_norm * rj_norm * rk_norm + ri_norm * rjrk_dot + rj_norm * rkri_dot + rk_norm * rirj_dot;
    
    /* w_face.resize(num.rows(), 1); */
    // return by reference
    w_face = 2.0 * num.binaryExpr(den, [] (double a, double b) { return std::atan2(a,b);} );
    /* w_face = 2.0 * num.binaryExpr(den, std::ptr_fun(::atan2)); */

    return 0;
    
}

int edge_factor(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& r_v, 
                const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& e1,
                const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& e2,
                const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& e3,
                const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 2> >& e1_vertex_map,
                const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 2> >& e2_vertex_map,
                const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 2> >& e3_vertex_map,
                Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > L1_edge,
                Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > L2_edge,
                Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > L3_edge) {

        Eigen::Array<double, Eigen::Dynamic, 3> r1i, r1j;
        r1i.resize(e1_vertex_map.rows(), 3);
        r1j.resize(e1_vertex_map.rows(), 3);
        for (int ii = 0; ii < e1_vertex_map.rows(); ++ii) {
            r1i.row(ii) = r_v.row(e1_vertex_map(ii, 0));
            r1j.row(ii) = r_v.row(e1_vertex_map(ii, 1));
        }
        Eigen::Array<double, Eigen::Dynamic, 1> r1i_norm, r1j_norm, e1_norm;
        r1i_norm.resize(r1i.rows(), 1);
        r1j_norm.resize(r1j.rows(), 1);
        e1_norm.resize(e1.rows(), 1);
        L1_edge.resize(e1.rows(), 1);

        r1i_norm = r1i.matrix().rowwise().norm();
        r1j_norm = r1j.matrix().rowwise().norm();
        e1_norm  = e1.matrix().rowwise().norm();
        L1_edge = (r1i_norm + r1j_norm + e1_norm) / (r1i_norm + r1j_norm - e1_norm);
        std::cout << e1_norm << std::endl;

}



