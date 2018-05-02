#include "potential.hpp"

#include <igl/sort.h>
#include <igl/unique_rows.h>
#include <igl/cross.h>
#include <igl/slice.h>
#include <igl/find.h>
#include <igl/dot_row.h>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/StdVector>

#include <algorithm>
#include <iostream>
#include <vector>
#include <tuple>
#include <cassert>
#include <cmath>

// MeshParam member functions
MeshParam::MeshParam(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& V_in,
                     const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 3> >& F_in) {
    V = V_in;
    F = F_in;
    polyhedron_parameters();
    face_dyad();
    edge_dyad();
}

void MeshParam::polyhedron_parameters( void ) {
    
    num_v = V.rows();
    num_f = F.rows();
    num_e = 3 * (num_v -2);

    // calculate all the edges zero based. This assumes it's already subtracted from the original OBJ for example
    Fa = F.col(0);
    Fb = F.col(1);
    Fc = F.col(2);

    igl::slice(V, Fa, (Eigen::Vector3i() << 0, 1, 2).finished(), V1);
    igl::slice(V, Fb, (Eigen::Vector3i() << 0, 1, 2).finished(), V2);
    igl::slice(V, Fc, (Eigen::Vector3i() << 0, 1, 2).finished(), V3);
    
    // compute the edge vectors
    e1 = V2 - V1;
    e2 = V3 - V2;
    e3 = V1 - V3;
    
    // vertex map
    e_vertex_map_stacked.resize(3 * num_f, 2);
    e_vertex_map_sorted.resize(3 * num_f, 2);
    
    e1_vertex_map.resize(num_f, Eigen::NoChange);
    e2_vertex_map.resize(num_f, Eigen::NoChange);
    e3_vertex_map.resize(num_f, Eigen::NoChange);

    e1_vertex_map.col(0) = Fb;
    e1_vertex_map.col(1) = Fa;

    e2_vertex_map.col(0) = Fc;
    e2_vertex_map.col(1) = Fb;

    e3_vertex_map.col(0) = Fa;
    e3_vertex_map.col(1) = Fc;

    e_vertex_map_stacked << e1_vertex_map, e2_vertex_map, e3_vertex_map;
    
    Eigen::MatrixXi IA, IC, sort_index;
    igl::sort(e_vertex_map_stacked, 2, true, e_vertex_map_sorted, sort_index);
    igl::unique_rows(e_vertex_map_sorted, e_vertex_map, unique_index, IC);

    igl::cross(e1, e2, normal_face);
    normal_face.rowwise().normalize();
    
    igl::cross(e1, normal_face, e1_normal);
    e1_normal.rowwise().normalize();

    igl::cross(e2, normal_face, e2_normal);
    e2_normal.rowwise().normalize();

    igl::cross(e3, normal_face, e3_normal);
    e3_normal.rowwise().normalize();
    
    // compute the centeroid of each face
    center_face = 1.0 / 3 * (V1 + V2 + V3);
    
    // edge vertex map
    edge_vertex_map = std::make_tuple(e1_vertex_map, e2_vertex_map, e3_vertex_map);

    // build vertex face map
    vf_map = vertex_face_map(V, F);
    
    e1_ind1b = vertex_map_search(e1_vertex_map, e1_vertex_map);
    e1_ind2b = vertex_map_search(e1_vertex_map, e2_vertex_map);
    e1_ind3b = vertex_map_search(e1_vertex_map, e3_vertex_map);

    e2_ind1b = vertex_map_search(e2_vertex_map, e1_vertex_map);
    e2_ind2b = vertex_map_search(e2_vertex_map, e2_vertex_map);
    e2_ind3b = vertex_map_search(e2_vertex_map, e3_vertex_map);
    
    e3_ind1b = vertex_map_search(e3_vertex_map, e1_vertex_map);
    e3_ind2b = vertex_map_search(e3_vertex_map, e2_vertex_map);
    e3_ind3b = vertex_map_search(e3_vertex_map, e3_vertex_map);
    // TODO make a vertex_map_inverse function
    
    Eigen::VectorXi faces_list(e1_ind1b.size());
    faces_list = Eigen::VectorXi::LinSpaced(e1_ind1b.size(), 0, e1_ind1b.size());
    
    e1_face_map.resize(num_f, 4);
    e2_face_map.resize(num_f, 4);
    e3_face_map.resize(num_f, 4);

    e1_face_map << faces_list, e1_ind1b, e1_ind2b, e1_ind3b;
    e2_face_map << faces_list, e2_ind1b, e2_ind2b, e2_ind3b;
    e3_face_map << faces_list, e3_ind1b, e3_ind2b, e3_ind3b;

    edge_face_map = std::make_tuple(e1_face_map, e2_face_map, e3_face_map);
}

void MeshParam::face_dyad( void ) {
    // compute the face dyad
    
    for (int ii = 0; ii < num_f; ++ii) {
        // outer product of normal_face vectors
        F_face.push_back(normal_face.row(ii).transpose() * normal_face.row(ii)); 
    }

}

void MeshParam::edge_dyad( void ) {
    // compute the edge dyad by looping
    Eigen::Matrix<double, 1, 3> nA1, nA2, nA3, nB1, nB2, nB3;
    Eigen::Matrix<double, 1, 3> nA, nB;

    Eigen::Matrix<int, 1, 3> invalid_row(3);
    invalid_row.fill(-1);

    for (int ii = 0; ii < num_f; ++ii) {
        // pick out the normals for the edges of the current face
        nA = normal_face.row(ii);

        nA1 = e1_normal.row(e1_face_map(ii, 0));
        nA2 = e2_normal.row(e2_face_map(ii, 0));
        nA3 = e3_normal.row(e3_face_map(ii, 0));
    
        // TODO edge_dyad loop need to find adjacent faces using e1_face_map
        // find the adjacent face for edge 1
        if (e1_face_map(ii, 1) != -1) { // adjacent face is also edge 1
            nB1 = e1_normal.row(e1_face_map(ii, 1)); 
            nB = normal_face.row(e1_face_map(ii, 1));
        } else if( e1_face_map(ii, 2) != -1) { // adjacent face is edge 2
            nB1 = e2_normal.row(e1_face_map(ii, 2));
            nB = normal_face.row(e1_face_map(ii, 2));
        } else if( e1_face_map(ii, 3) != -1) { // adjacent face is edge 3
            nB1 = e3_normal.row(e1_face_map(ii, 3));
            nB = normal_face.row(e1_face_map(ii, 3));
        }
        
        // compute the edge dyad
        E1_edge.push_back(nA.transpose() * nA1 + nB.transpose() * nB1);

        // find adjacent edge for edge 2
        if (e2_face_map(ii, 1) != -1 ){
            nB2 = e1_normal.row(e2_face_map(ii, 1));
            nB = normal_face.row(e2_face_map(ii, 1));
        } else if(e2_face_map(ii, 2) != -1) {
            nB2 = e2_normal.row(e2_face_map(ii, 2));
            nB = normal_face.row(e2_face_map(ii, 2));
        } else if (e2_face_map(ii, 3) != -1) {
            nB2 = e3_normal.row(e2_face_map(ii, 3));
            nB = normal_face.row(e2_face_map(ii, 3));
        }
        
        // second edge dyad
        E2_edge.push_back(nA.transpose() * nA2 + nB.transpose() * nB2);
        
        // find the adjacent edge for edge 3
        if (e3_face_map(ii, 1) != -1 ) {
            nB3 = e1_normal.row(e3_face_map(ii, 1));
            nB = normal_face.row(e3_face_map(ii, 1));
        } else if (e3_face_map(ii, 2) != -1 ) {
            nB3 = e2_normal.row(e3_face_map(ii, 2));
            nB = normal_face.row(e3_face_map(ii, 2));
        } else if (e3_face_map(ii, 3) != -1 ) {
            nB3 = e3_normal.row(e3_face_map(ii, 3));
            nB = normal_face.row(e3_face_map(ii, 3));
        }

        E3_edge.push_back(nA.transpose() * nA3 + nB.transpose() * nB3);
    }
}

// Asteroid class
Asteroid::Asteroid(MeshParam& mesh_param_in) {
    mesh_param = std::make_shared<MeshParam>(mesh_param_in);
}

Asteroid::Asteroid(std::shared_ptr<MeshParam> mesh_param_in) {
    mesh_param = mesh_param_in;
}

void Asteroid::polyhedron_potential(const Eigen::Ref<const Eigen::Vector3d>& state) {

    Eigen::Matrix<double, Eigen::Dynamic, 3> r_v = mesh_param->V.matrix().rowwise() - state.transpose();
    
    // Compute w_face using laplacian_factor
    Eigen::Matrix<double, Eigen::Dynamic, 1> w_face = laplacian_factor(r_v, mesh_param->Fa, mesh_param->Fb, mesh_param->Fc);

    if (std::abs(w_face.sum()) < 1e-10) {
        std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> L_all =
            edge_factor(r_v, mesh_param->e1, mesh_param->e2, mesh_param->e3,
                    mesh_param->e1_vertex_map, mesh_param->e2_vertex_map,
                    mesh_param->e3_vertex_map);
            
            // face contribution
            //
            // edge contribution
    } else {
        // set everything to zero
    }
    // loop over the faces and face dyads
    //
    // loop over edges and edge dyads
    // use unique_index to slice out the edge dyads we need
    // TODO
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
void face_contribution(const Eigen::Ref<) {
    
    std::cout << r_v << std::endl;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> laplacian_factor(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v,
                     const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 1> >& Fa,
                     const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 1> >& Fb,
                     const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 1> >& Fc) {
    // form the ri, rj, rk arrays
    Eigen::MatrixXd ri(Fa.rows(), 3), rj(Fa.rows(), 3), rk(Fa.rows(), 3), rjrk_cross(Fa.rows(), 3);
    
    igl::slice(r_v, Fa, (Eigen::Vector3d() << 0, 1, 2).finished(), ri);
    igl::slice(r_v, Fb, (Eigen::Vector3d() << 0, 1, 2).finished(), rj);
    igl::slice(r_v, Fc, (Eigen::Vector3d() << 0, 1, 2).finished(), rk);
    
    igl::cross(rj, rk, rjrk_cross);
    
    Eigen::Matrix<double, Eigen::Dynamic, 1> ri_norm(Fa.rows(), 1), rj_norm(Fa.rows(), 1), rk_norm(Fa.rows(), 1);

    ri_norm = ri.rowwise().norm();
    rj_norm = rj.rowwise().norm();
    rk_norm = rk.rowwise().norm();
    
    // dot product terms
    Eigen::MatrixXd rjrk_dot(Fa.rows(), 1), rkri_dot(Fa.rows(), 1), rirj_dot(Fa.rows(), 1);

    rjrk_dot = igl::dot_row(rj, rk);
    rkri_dot = igl::dot_row(rk, ri);
    rirj_dot = igl::dot_row(ri, rj);
    
    // numerator and denonminator of atan2
    Eigen::Matrix<double, Eigen::Dynamic, 1> num(Fa.rows(), 1), den(Fa.rows(), 1);
    num = (ri.array() * rjrk_cross.array()).rowwise().sum();
    den = ri_norm.array() * rj_norm.array() * rk_norm.array() + ri_norm.array() * rjrk_dot.array() + rj_norm.array() * rkri_dot.array() + rk_norm.array() * rirj_dot.array();
    
    // return by reference
    Eigen::Matrix<double, Eigen::Dynamic, 1> w_face(Fa.rows(), 1);
    w_face = 2.0 * num.binaryExpr(den, [] (double a, double b) { return std::atan2(a,b);} );
    /* w_face = 2.0 * num.binaryExpr(den, std::ptr_fun(::atan2)); */

    return w_face;
    
}

// This is slower than numpy
std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> edge_factor(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v, 
                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& e1,
                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& e2,
                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& e3,
                const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& e1_vertex_map,
                const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& e2_vertex_map,
                const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& e3_vertex_map) {
    
    const int num_f = e1_vertex_map.rows();

    Eigen::MatrixXd r1i(num_f, 3), r1j(num_f, 3),
                    r2i(num_f, 3), r2j(num_f, 3),
                    r3i(num_f, 3), r3j(num_f, 3);
    
    igl::slice(r_v, e1_vertex_map.col(0), 1, r1i);
    igl::slice(r_v, e1_vertex_map.col(1), 1, r1j);
    
    igl::slice(r_v, e2_vertex_map.col(0), 1, r2i);
    igl::slice(r_v, e2_vertex_map.col(1), 1, r2j);

    igl::slice(r_v, e3_vertex_map.col(0), 1, r3i);
    igl::slice(r_v, e3_vertex_map.col(1), 1, r3j);
    
    Eigen::VectorXd L1_edge(num_f), L2_edge(num_f), L3_edge(num_f);

    Eigen::VectorXd r1i_norm(num_f), r1j_norm(num_f), e1_norm(num_f);

    r1i_norm = r1i.rowwise().norm();
    r1j_norm = r1j.rowwise().norm();
    e1_norm  = e1.rowwise().norm();
    L1_edge = Eigen::log((r1i_norm.array() + r1j_norm.array() + e1_norm.array()) / (r1i_norm.array() + r1j_norm.array() - e1_norm.array()));

    Eigen::Matrix<double, Eigen::Dynamic, 1> r2i_norm(num_f), r2j_norm(num_f), e2_norm(num_f);
    r2i_norm = r2i.rowwise().norm();
    r2j_norm = r2j.rowwise().norm();
    e2_norm  = e2.rowwise().norm();
    L2_edge = Eigen::log((r2i_norm.array() + r2j_norm.array() + e2_norm.array()) / (r2i_norm.array() + r2j_norm.array() - e2_norm.array()));

    Eigen::Matrix<double, Eigen::Dynamic, 1> r3i_norm(num_f), r3j_norm(num_f), e3_norm(num_f);
    r3i_norm = r3i.matrix().rowwise().norm();
    r3j_norm = r3j.matrix().rowwise().norm();
    e3_norm  = e3.matrix().rowwise().norm();
    L3_edge = Eigen::log((r3i_norm.array() + r3j_norm.array() + e3_norm.array()) / (r3i_norm.array() + r3j_norm.array() - e3_norm.array()));

    return std::make_tuple(L1_edge, L2_edge, L3_edge);
}

std::tuple<Eigen::VectorXi, Eigen::VectorXi> search_index(const Eigen::Ref<const Eigen::VectorXi>& a, const Eigen::Ref<const Eigen::VectorXi>& b) {
    std::size_t lena(a.size()), lenb(b.size());

    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ae(lena, lenb), be(lena, lenb);
    
    ae = b.rowwise().replicate(lena).transpose();
    be = a.rowwise().replicate(lenb);

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> equal_mat = ae.array() == be.array();
    Eigen::SparseMatrix<bool, Eigen::RowMajor> sparse = equal_mat.sparseView();
    
    std::vector<int> row, col;
    // iterate over the sparse matrix
    for (int ii=0; ii < sparse.outerSize(); ++ii) {
        for (Eigen::SparseMatrix<bool, Eigen::RowMajor>::InnerIterator it(sparse,ii); it; ++it) {
            /* it.value(); */
            /* it.row();   // row index */
            /* it.col();   // col index (here it is equal to k) */
            /* it.index(); // inner index, here it is equal to it.row() */
            row.push_back(it.row());
            col.push_back(it.col());
            /* std::cout << "(" << it.row() << ","; // row index */
            /* std::cout << it.col() << ")\n"; // col index (here it is equal to k) */
        }
    }
    Eigen::VectorXi inda1(row.size()), indb1(col.size());
    inda1 = Eigen::Map<Eigen::VectorXi>(row.data(), row.size());
    indb1 = Eigen::Map<Eigen::VectorXi>(col.data(), col.size());
    return std::make_tuple(inda1, indb1);
}

Eigen::VectorXi vertex_map_search(const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& a_map, 
        const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& b_map) { 
    std::tuple<Eigen::VectorXi, Eigen::VectorXi> inda1_indb1 = search_index(a_map.col(0), b_map.col(1));
    
    Eigen::VectorXi inda1, indb1;
    inda1 = std::get<0>(inda1_indb1);
    indb1 = std::get<1>(inda1_indb1);
    
    int invalid = -1;
    Eigen::VectorXi index_map(a_map.rows());
    index_map.fill(invalid);

    Eigen::MatrixXi amap_inda1, bmap_indb1;
    
    igl::slice(a_map, std::get<0>(inda1_indb1), 1, amap_inda1); // want column 1
    igl::slice(b_map, std::get<1>(inda1_indb1), 1, bmap_indb1); // want column 1
    
    Eigen::Matrix<bool, Eigen::Dynamic, 1> index_match;
    index_match = amap_inda1.array().col(1) == bmap_indb1.array().col(0);
    Eigen::SparseVector<bool> sparse = index_match.sparseView();
    
    std::vector<int> index_match_vec;
    // iterate over the sparse vector again
    for (Eigen::SparseVector<bool>::InnerIterator it(sparse); it; ++it){
        index_match_vec.push_back(it.index());
    }
    Eigen::VectorXi index_match_map = Eigen::Map<Eigen::VectorXi>(index_match_vec.data(), index_match_vec.size());
    
    // now loop and create index_map vector
    for (int ii=0; ii < index_match_map.size(); ++ii) {
        index_map(inda1(index_match_map(ii))) = indb1(index_match_map(ii));
    }
    
    return index_map;
}
