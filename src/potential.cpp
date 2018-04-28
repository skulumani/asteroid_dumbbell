#include "potential.hpp"

#include <igl/sort.h>
#include <igl/unique_rows.h>
#include <igl/cross.h>
#include <igl/slice.h>
#include <igl/find.h>

#include <Eigen/SparseCore>

#include <algorithm>
#include <iostream>
#include <vector>
#include <tuple>


// TODO Need to make this function
void polyhedron_parameters(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> > & V,
        const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 3> >& F) {
    
    std::size_t num_v = V.rows();
    std::size_t num_f = F.rows();
    std::size_t num_e = 3 * (num_v -2);

    // calculate all the edges zero based. This assumes it's already subtracted from the original OBJ for example
    Eigen::Matrix<int, Eigen::Dynamic, 1> Fa, Fb, Fc;
    Fa = F.col(0);
    Fb = F.col(1);
    Fc = F.col(2);

    Eigen::Matrix<double, Eigen::Dynamic, 3> V1(num_f, 3), V2(num_f, 3), V3(num_f, 3);

    igl::slice(V, Fa, (Eigen::Vector3i() << 0, 1, 2).finished(), V1);
    igl::slice(V, Fb, (Eigen::Vector3i() << 0, 1, 2).finished(), V2);
    igl::slice(V, Fc, (Eigen::Vector3i() << 0, 1, 2).finished(), V3);
    
    // compute the edge vectors
    Eigen::Matrix<double, Eigen::Dynamic, 3> e1(num_f, 3), e2(num_f, 3), e3(num_f, 3);
    e1 = V2 - V1;
    e2 = V3 - V2;
    e3 = V1 - V3;
    
    // vertex map
    Eigen::Matrix<int, Eigen::Dynamic, 2> e1_vertex_map(num_f, 2),
                                          e2_vertex_map(num_f, 2), 
                                          e3_vertex_map(num_f, 2),
                                          e_vertex_map_stacked(3 * num_f, 2),
                                          e_vertex_map_sorted(3 * num_f, 2);

    e1_vertex_map.col(0) = Fb;
    e1_vertex_map.col(1) = Fa;

    e2_vertex_map.col(0) = Fc;
    e2_vertex_map.col(1) = Fb;

    e3_vertex_map.col(0) = Fa;
    e3_vertex_map.col(1) = Fc;
    
    e_vertex_map_stacked << e1_vertex_map, e2_vertex_map, e3_vertex_map;
    
    Eigen::MatrixXi unique_index, IA, IC, e_vertex_map;

    igl::sort(e_vertex_map_stacked, 2, true, e_vertex_map_sorted, unique_index);
    igl::unique_rows(e_vertex_map_sorted, e_vertex_map, unique_index, IC);


    Eigen::Matrix<double, Eigen::Dynamic, 3> normal_face(num_f, 3),
                                             e1_normal(num_f, 3),
                                             e2_normal(num_f, 3),
                                             e3_normal(num_f, 3);

    igl::cross(e1, e2, normal_face);
    normal_face.rowwise().normalize();
    
    igl::cross(e1, normal_face, e1_normal);
    e1_normal.rowwise().normalize();

    igl::cross(e2, normal_face, e2_normal);
    e2_normal.rowwise().normalize();

    igl::cross(e3, normal_face, e3_normal);
    e3_normal.rowwise().normalize();
    
    // compute the centeroid of each face
    Eigen::Matrix<double, Eigen::Dynamic, 3> center_face(num_f, 3);
    center_face = 1.0 / 3 * (V1 + V2 + V3);
    
    // edge vertex map
    std::tuple<Eigen::Matrix<int, Eigen::Dynamic, 2>, Eigen::Matrix<int, Eigen::Dynamic, 2>, Eigen::Matrix<int, Eigen::Dynamic, 2> > edge_vertex_map;
    edge_vertex_map = std::make_tuple(e1_vertex_map, e2_vertex_map, e3_vertex_map);

    // build vertex face map
    std::vector<std::vector<int> > vf_map = vertex_face_map(V, F);
    

    // TODO Need to searching to find matching edges (search_edge_vertex_map)


    // replicate the columns of e1_vertex_map
    std::size_t lena(e1_vertex_map.col(0).size()), lenb(e1_vertex_map.col(1).size());

    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ae(lena, lenb), be(lena, lenb);

    ae = e1_vertex_map.col(1).rowwise().replicate(lena).transpose();
    be = e1_vertex_map.col(0).rowwise().replicate(lenb);
    
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> equal_mat = ae.array() == be.array();
    Eigen::SparseMatrix<bool> sparse(lena, lenb);
    sparse = equal_mat.sparseView();

    Eigen::VectorXi I, J, index;
    igl::find(sparse, I, J, index);

    std::cout << index << std::endl;
}

std::vector<std::vector<int> > vertex_face_map(const Eigen::Ref<const Eigen::MatrixXd> & V, const Eigen::Ref<const Eigen::MatrixXi> &F) {

    std::vector<std::vector<int> > vf_map(V.rows());
    // loop over faces in F array
    for (int ii = 0; ii < F.rows(); ++ii) {
        for (int jj = 0; jj < 3; ++jj) {
            vf_map[F(ii,jj)].push_back(ii);
        }
    }

    return vf_map;
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

// This is slower than numpy
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
    
        Eigen::Array<double, Eigen::Dynamic, 3> r1i, r1j, r2i, r2j, r3i, r3j;
        r1i.resize(e1_vertex_map.rows(), 3);
        r1j.resize(e1_vertex_map.rows(), 3);

        r2i.resize(e2_vertex_map.rows(), 3);
        r2j.resize(e2_vertex_map.rows(), 3);

        r3i.resize(e3_vertex_map.rows(), 3);
        r3j.resize(e3_vertex_map.rows(), 3);
        for (int ii = 0; ii < e1_vertex_map.rows(); ++ii) {
            r1i.row(ii) = r_v.row(e1_vertex_map(ii, 0));
            r1j.row(ii) = r_v.row(e1_vertex_map(ii, 1));

            r2i.row(ii) = r_v.row(e2_vertex_map(ii, 0));
            r2j.row(ii) = r_v.row(e2_vertex_map(ii, 1));

            r3i.row(ii) = r_v.row(e3_vertex_map(ii, 0));
            r3j.row(ii) = r_v.row(e3_vertex_map(ii, 1));
        }

        Eigen::Array<double, Eigen::Dynamic, 1> r1i_norm, r1j_norm, e1_norm;
        r1i_norm = r1i.matrix().rowwise().norm();
        r1j_norm = r1j.matrix().rowwise().norm();
        e1_norm  = e1.matrix().rowwise().norm();
        L1_edge = Eigen::log((r1i_norm + r1j_norm + e1_norm) / (r1i_norm + r1j_norm - e1_norm));

        Eigen::Array<double, Eigen::Dynamic, 1> r2i_norm, r2j_norm, e2_norm;
        r2i_norm = r2i.matrix().rowwise().norm();
        r2j_norm = r2j.matrix().rowwise().norm();
        e2_norm  = e2.matrix().rowwise().norm();
        L2_edge = Eigen::log((r2i_norm + r2j_norm + e2_norm) / (r2i_norm + r2j_norm - e2_norm));

        Eigen::Array<double, Eigen::Dynamic, 1> r3i_norm, r3j_norm, e3_norm;
        r3i_norm = r3i.matrix().rowwise().norm();
        r3j_norm = r3j.matrix().rowwise().norm();
        e3_norm  = e3.matrix().rowwise().norm();
        L3_edge = Eigen::log((r3i_norm + r3j_norm + e3_norm) / (r3i_norm + r3j_norm - e3_norm));

        return 0;
}

std::tuple<Eigen::VectorXi, Eigen::VectorXi> search_index(const Eigen::Ref<const Eigen::VectorXi>& a, const Eigen::Ref<const Eigen::VectorXi>& b) {
    std::size_t lena(a.size()), lenb(b.size());
}

