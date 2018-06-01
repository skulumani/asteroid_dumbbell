#include "potential.hpp"
#include "mesh.hpp"
#include "reconstruct.hpp"

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
#include <string>
#include <stdexcept>
#include <memory>

#include <omp.h>

// **********************Forward declaration************************
std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 3>, 
           Eigen::Matrix<double, Eigen::Dynamic, 3>,
           Eigen::Matrix<double, Eigen::Dynamic, 3> > mesh_edges(const Eigen::Ref<const Eigen::MatrixXd>& V, 
                                                                 const Eigen::Ref<const Eigen::MatrixXi>& F);

// ***************** MeshParam ***************************************
// Constructors
MeshParam::MeshParam(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& V_in,
                     const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 3> >& F_in) {
	mesh = std::make_shared<MeshData>(V_in, F_in);

    polyhedron_parameters();
    face_dyad();
    edge_dyad();
}

MeshParam::MeshParam(std::shared_ptr<MeshData> mesh_in) {
	mesh = mesh_in;

    polyhedron_parameters();
    face_dyad();
    edge_dyad();
}

void MeshParam::polyhedron_parameters( void ) {
    
    const Eigen::MatrixXd& V = mesh->get_verts();
    const Eigen::MatrixXi& F = mesh->get_faces();

    num_v = V.rows();
    num_f = F.rows();
    num_e = 3 * (num_v -2);

    // calculate all the edges zero based. This assumes it's already subtracted from the original OBJ for example
    Fa = F.col(0);
    Fb = F.col(1);
    Fc = F.col(2);
    
    Eigen::Matrix<double, Eigen::Dynamic, 3> V1, V2, V3;
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
    
}

void MeshParam::face_dyad( void ) {
    // compute the face dyad
   	F_face.resize(num_f);
    
    #pragma omp parallel for
    for (int ii = 0; ii < num_f; ++ii) {
        // outer product of normal_face vectors
        F_face[ii] = normal_face.row(ii).transpose() * normal_face.row(ii);
    }

}

void MeshParam::edge_dyad( void ) {
    // compute the edge dyad by looping
    Eigen::Matrix<double, 1, 3> nA1, nA2, nA3, nB1, nB2, nB3;
    Eigen::Matrix<double, 1, 3> nA, nB;

    Eigen::Matrix<int, 1, 3> invalid_row(3);
    invalid_row.fill(-1);
	
	E1_edge.resize(num_f);
	E2_edge.resize(num_f);
	E3_edge.resize(num_f);	
    
    // generate the edge face/vertex maps
    Eigen::VectorXi e1_ind1b(num_f), e1_ind2b(num_f), e1_ind3b(num_f),
                    e2_ind1b(num_f), e2_ind2b(num_f), e2_ind3b(num_f),
                    e3_ind1b(num_f), e3_ind2b(num_f), e3_ind3b(num_f);

    Eigen::MatrixXi e1_face_map(num_f, 4), e2_face_map(num_f, 4), e3_face_map(num_f, 4);

    Eigen::VectorXi faces_list(num_f);
    faces_list = Eigen::VectorXi::LinSpaced(e1_ind1b.size(), 0, e1_ind1b.size());
    
    #pragma omp parallel private(nA, nA1, nA2, nA3, nB1, nB2, nB3, nB) 
    #pragma omp single
    {
    
        #pragma omp task depend(out:e1_ind1b)
        {
            e1_ind1b = vertex_map_search(e1_vertex_map, e1_vertex_map);
        }

        #pragma omp task depend(out:e1_ind2b)
        {
            e1_ind2b = vertex_map_search(e1_vertex_map, e2_vertex_map);
        }
        #pragma omp task depend(out:e1_ind3b)
        {
            e1_ind3b = vertex_map_search(e1_vertex_map, e3_vertex_map);
        }

        #pragma omp task depend(out:e1_face_map) depend(in:e1_ind1b) depend(in:e1_ind2b) depend(in:e1_ind3b)
        {
            e1_face_map << faces_list, e1_ind1b, e1_ind2b, e1_ind3b;
        }

        
        #pragma omp task depend(out:e2_ind1b)
        {
            e2_ind1b = vertex_map_search(e2_vertex_map, e1_vertex_map);
        }

        #pragma omp task depend(out:e2_ind2b)
        {
            e2_ind2b = vertex_map_search(e2_vertex_map, e2_vertex_map);
        }

        #pragma omp task depend(out:e2_ind3b)
        {
            e2_ind3b = vertex_map_search(e2_vertex_map, e3_vertex_map);
        }

        #pragma omp task depend(out:e2_face_map) depend(in:e2_ind1b) depend(in:e2_ind2b) depend(in:e2_ind3b)
        {
            e2_face_map << faces_list, e2_ind1b, e2_ind2b, e2_ind3b;
        }

        #pragma omp task depend(out:e3_ind1b)
        {
            e3_ind1b = vertex_map_search(e3_vertex_map, e1_vertex_map);
        }

        #pragma omp task depend(out:e3_ind2b)
        {
            e3_ind2b = vertex_map_search(e3_vertex_map, e2_vertex_map);
        }

        #pragma omp task depend(out:e3_ind3b)
        {
            e3_ind3b = vertex_map_search(e3_vertex_map, e3_vertex_map);
        }

        #pragma omp task depend(out:e3_face_map) depend(in:e3_ind1b) depend(in:e3_ind2b) depend(in:e3_ind3b)
        {
            e3_face_map << faces_list, e3_ind1b, e3_ind2b, e3_ind3b;
        }

    // E1_edge
    #pragma omp task depend(in:e1_face_map)
    {
        #pragma omp parallel for
        for (int ii = 0; ii < num_f; ++ii) {
            // pick out the normals for the edges of the current face
            nA = normal_face.row(ii);

            nA1 = e1_normal.row(e1_face_map(ii, 0));

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
            E1_edge[ii] = nA.transpose() * nA1 + nB.transpose() * nB1;

        }
    }
    
    #pragma omp task depend(in:e2_face_map)
    {
        #pragma omp parallel for
        for (int ii = 0; ii < num_f; ++ii) {
            // pick out the normals for the edges of the current face
            nA = normal_face.row(ii);

            nA2 = e2_normal.row(e2_face_map(ii, 0));

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
            E2_edge[ii] = nA.transpose() * nA2 + nB.transpose() * nB2;

        }
    }

    #pragma omp task depend(in:e3_face_map)
    {
        #pragma omp parallel for
        for (int ii = 0; ii < num_f; ++ii) {
            // pick out the normals for the edges of the current face
            nA = normal_face.row(ii);

            nA3 = e3_normal.row(e3_face_map(ii, 0));
        
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

            E3_edge[ii] = nA.transpose() * nA3 + nB.transpose() * nB3;
        }
    }

    }
}

void MeshParam::update_mesh(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& V_in,
                            const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 3> >& F_in) {
    
    // update the connected meshdata
    mesh->update_mesh(V_in, F_in);
    // update all the mesharameters
    polyhedron_parameters();
    face_dyad();
    edge_dyad();
}
// ************************ Asteroid class ************************************
Asteroid::Asteroid(const std::string& name_in, MeshParam& mesh_param_in) {
    mesh_param = std::make_shared<MeshParam>(mesh_param_in);
    name = name_in;
    init_asteroid();
}

Asteroid::Asteroid(const std::string& name_in,
                   const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& V_in,
                   const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 3> >& F_in) {
    mesh_param = std::make_shared<MeshParam>(V_in, F_in);
    name = name_in;
    init_asteroid();
}

Asteroid::Asteroid(const std::string& name_in, std::shared_ptr<MeshParam> mesh_param_in) {
    mesh_param = mesh_param_in;
    name = name_in;
    init_asteroid();
}

Asteroid::Asteroid(const std::string& name_in, 
                   std::shared_ptr<ReconstructMesh> rmesh_in) {
    mesh_param = std::make_shared<MeshParam>(rmesh_in->get_mesh());
    name = name_in;
    init_asteroid();
}

Asteroid::Asteroid(const std::string& name_in,
                   std::shared_ptr<MeshData> mesh_in) {
    mesh_param = std::make_shared<MeshParam>(mesh_in);
    name = name_in;
    init_asteroid();
}

void Asteroid::init_asteroid( void ) {
    const double kPI = 3.141592653589793;

    // construct some of the data for the asteroid
    if (name.compare("castalia") == 0) {
        sigma = 2.1;
        axes << 1.6130, 0.9810, 0.8260;
        axes = axes.array() / 2.0;
        omega = 2 * kPI / 4.07 / 3600.0;
        M = 1.4091e12;
    } else if (name.compare("itokawa") == 0) {
        M = 3.51e10;
        sigma = 1.9;
        axes << 535, 294, 209;
        axes = axes.array() / 2.0 / 1.0e3;
        omega = 2 * kPI / 12.132 / 3600.0;
    } else if (name.compare("eros") == 0) {
        M = 4.463e-4 / G;
        sigma = 2.67;
        axes << 34.4, 11.7, 11.7;
        omega = 2 * kPI / 5.27 / 3600;
    } else if (name.compare("cube") == 0) {
        M = 1;
        sigma = 1;
        axes << 1, 1, 1;
        omega = 1;
    } else {
        throw std::invalid_argument( "Invalid asteroid name" );
    }
    
    // convert density to kg/km^3
    sigma = sigma / 1000.0 * pow(100.0, 3) * pow(1000.0, 3);
}

void Asteroid::polyhedron_potential(const Eigen::Ref<const Eigen::Vector3d>& state) {
    
    // state position should be in the asteroid fixed frame
    Eigen::Matrix<double, Eigen::Dynamic, 3> r_v = mesh_param->mesh->get_verts().rowwise() - state.transpose();
    
    // Compute w_face using laplacian_factor
    Eigen::Matrix<double, Eigen::Dynamic, 1> w_face = laplacian_factor(r_v);
    
    if (std::abs(w_face.sum()) < 1e-10) {
        std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> L_all =
            edge_factor(r_v);
        
        std::tuple<double, Eigen::Matrix<double, 3, 1>, Eigen::Matrix<double, 3, 3> > face_grav, edge_grav;

        #pragma omp parallel shared(r_v)
        {
            #pragma omp task
            {
                // face contribution
                face_grav = face_contribution(r_v, w_face);
            }
            #pragma omp task
            {
            // edge contribution
            edge_grav = edge_contribution(r_v, L_all);
            }
        }
        
        // combine them both
        U = 1.0 / 2.0 * G * sigma * (std::get<0>(edge_grav) - std::get<0>(face_grav));
        U_grad = G * sigma * (-std::get<1>(edge_grav) + std::get<1>(face_grav));
        U_grad_mat = G * sigma * (std::get<2>(edge_grav) - std::get<2>(face_grav));
        Ulaplace = -G * sigma * w_face.sum();

    } else {
        U = 0;
        U_grad.setZero();
        U_grad_mat.setZero();
        Ulaplace = 0;
    }

}

Eigen::Matrix<double, Eigen::Dynamic, 3> Asteroid::rotate_vertices(const double& time) const {
    
    // define the rotation matrix Ra
    Eigen::Matrix<double, 3, 3> Ra;
    Ra = Eigen::AngleAxis<double>(omega * time, (Eigen::Vector3d() << 0, 0, 1).finished());
    
    // multiply times all the vertices
    Eigen::Matrix<double, Eigen::Dynamic, 3> nv(mesh_param->num_v, 3);
    nv = (Ra * mesh_param->mesh->get_verts().transpose()).transpose();

    return nv;
}

Eigen::Matrix<double, 3, 3> Asteroid::rot_ast2int(const double& time) {
    Eigen::Matrix<double, 3, 3> Ra;
    Ra = Eigen::AngleAxis<double>(omega * time, (Eigen::Vector3d() << 0 ,0, 1).finished());

    return Ra;
}

void Asteroid::update_rotation(const double& time) {
    // get the current Ra
    Eigen::Matrix<double, 3, 3> Ra = rot_ast2int(time);
    // rotate asteroid vertices
    Eigen::Matrix<double, Eigen::Dynamic, 3> nv(mesh_param->num_v, 3);
    Eigen::Matrix<int, Eigen::Dynamic, 3> nf(mesh_param->num_f, 3);

    nv = (Ra * mesh_param->get_verts().transpose()).transpose();
    nf = mesh_param->get_faces();
    // update the meshdata (vertices)
    mesh_param->update_mesh(nv, nf); 
    // update meshparam and asteroid parameters
    init_asteroid();
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
std::tuple<double, Eigen::Matrix<double, 3, 1>, Eigen::Matrix<double, 3, 3> > Asteroid::face_contribution(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v,
        const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1> >& w_face) {
    
    const Eigen::Matrix<int, Eigen::Dynamic, 1>& Fa = mesh_param->mesh->get_faces().col(0);

    const std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3> > >& F_face = mesh_param->F_face;

    const int num_f = Fa.rows();

    Eigen::Matrix<double, Eigen::Dynamic, 3> ra(num_f, 3);
    igl::slice(r_v, Fa, 1, ra);
    // find ra dot with F_face
    double U_face = 0;
    Eigen::Matrix<double, 3, 1> U_grad_face(3);
    U_grad_face.setZero();
    Eigen::Matrix<double, 3, 3> U_grad_mat_face(3, 3);
    U_grad_mat_face.setZero();
    
    #pragma omp declare reduction (merge : Eigen::Matrix<double, 3, 1> : omp_out += omp_in)
    #pragma omp declare reduction (merge : Eigen::Matrix<double, 3, 3> : omp_out += omp_in)

    #pragma omp parallel for reduction(+: U_face) reduction(merge: U_grad_face) reduction(merge: U_grad_mat_face)
    for (int ii = 0; ii < num_f; ++ii) {
        U_face += (ra.row(ii) * F_face[ii] * ra.row(ii).transpose() * w_face(ii)).value(); 
        U_grad_face += F_face[ii] * ra.row(ii).transpose() * w_face(ii); 
        U_grad_mat_face += F_face[ii] * w_face(ii);
    }
    
    return std::make_tuple(U_face, U_grad_face, U_grad_mat_face);
}

std::tuple<double, Eigen::Matrix<double, 3, 1>, Eigen::Matrix<double, 3, 3> > Asteroid::edge_contribution(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v,
        const std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>& L_tuple) {

    // redefine some variables for local use
    const Eigen::MatrixXi& e_vertex_map = mesh_param->e_vertex_map;
    const Eigen::MatrixXi& unique_index = mesh_param->unique_index;
    const std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3> > >& E1_edge = mesh_param->E1_edge;
    const std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3> > >& E2_edge = mesh_param->E2_edge;
    const std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3> > >& E3_edge = mesh_param->E3_edge;

    // edge contribution
    // combine all the Ei and Li into big vectors then pick out the unique ones
    std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3> > > E_all(3 * std::get<0>(L_tuple).rows());
    E_all.assign(E1_edge.begin(), E1_edge.end());
    E_all.insert(E_all.end(), E2_edge.begin(), E2_edge.end());
    E_all.insert(E_all.end(), E3_edge.begin(), E3_edge.end());
    
    Eigen::VectorXd L_all(3 * std::get<0>(L_tuple).rows());
    L_all << std::get<0>(L_tuple), std::get<1>(L_tuple), std::get<2>(L_tuple);
        
    Eigen::Matrix<double, Eigen::Dynamic, 3> re(e_vertex_map.rows(), 3);
    igl::slice(r_v, e_vertex_map.col(0), 1, re);

    double U_edge = 0;
    Eigen::Matrix<double, 3, 1> U_grad_edge;
    U_grad_edge.setZero();
    Eigen::Matrix<double, 3, 3> U_grad_mat_edge;
    U_grad_mat_edge.setZero();
    
    int index = unique_index(0, 0);
    #pragma omp declare reduction (merge : Eigen::Matrix<double, 3, 1> : omp_out += omp_in)
    #pragma omp declare reduction (merge : Eigen::Matrix<double, 3, 3> : omp_out += omp_in)

    #pragma omp parallel for reduction(+: U_edge) reduction(merge: U_grad_edge) reduction(merge: U_grad_mat_edge)
    for (int ii = 0; ii < unique_index.size(); ++ii) {
        index = unique_index(ii,0);
        U_edge += (re.row(ii) * E_all[index] * re.row(ii).transpose() * L_all( index )).value();
        U_grad_edge += E_all[index] * re.row(ii).transpose() * L_all(index);
        U_grad_mat_edge += E_all[index] * L_all(index);
    }
    
    return std::make_tuple(U_edge, U_grad_edge, U_grad_mat_edge);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> Asteroid::laplacian_factor(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v) {

    const Eigen::MatrixXd& V = mesh_param->mesh->get_verts();
    const Eigen::MatrixXi& F = mesh_param->mesh->get_faces();
    
    const int num_f = F.rows();

    // form the ri, rj, rk arrays
    Eigen::MatrixXd ri(num_f, 3), rj(num_f, 3), rk(num_f, 3), rjrk_cross(num_f, 3);
    
    igl::slice(r_v, F.col(0), (Eigen::Vector3d() << 0, 1, 2).finished(), ri);
    igl::slice(r_v, F.col(1), (Eigen::Vector3d() << 0, 1, 2).finished(), rj);
    igl::slice(r_v, F.col(2), (Eigen::Vector3d() << 0, 1, 2).finished(), rk);
    
    igl::cross(rj, rk, rjrk_cross);
    
    Eigen::Matrix<double, Eigen::Dynamic, 1> ri_norm(num_f, 1), rj_norm(num_f, 1), rk_norm(num_f, 1);

    ri_norm = ri.rowwise().norm();
    rj_norm = rj.rowwise().norm();
    rk_norm = rk.rowwise().norm();
    
    // dot product terms
    Eigen::MatrixXd rjrk_dot(num_f, 1), rkri_dot(num_f, 1), rirj_dot(num_f, 1);

    rjrk_dot = igl::dot_row(rj, rk);
    rkri_dot = igl::dot_row(rk, ri);
    rirj_dot = igl::dot_row(ri, rj);
    
    // numerator and denonminator of atan2
    Eigen::Matrix<double, Eigen::Dynamic, 1> num(num_f, 1), den(num_f, 1);
    num = (ri.array() * rjrk_cross.array()).rowwise().sum();
    den = ri_norm.array() * rj_norm.array() * rk_norm.array() + ri_norm.array() * rjrk_dot.array() + rj_norm.array() * rkri_dot.array() + rk_norm.array() * rirj_dot.array();
    
    // return by reference
    Eigen::Matrix<double, Eigen::Dynamic, 1> w_face(num_f, 1);
    w_face = 2.0 * num.binaryExpr(den, [] (double a, double b) { return std::atan2(a,b);} );
    /* w_face = 2.0 * num.binaryExpr(den, std::ptr_fun(::atan2)); */

    return w_face;
    
}


// This is slower than numpy
std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> Asteroid::edge_factor(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v) {
    
    std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 3>,
               Eigen::Matrix<double, Eigen::Dynamic, 3>,
               Eigen::Matrix<double, Eigen::Dynamic, 3> > edges = mesh_edges(mesh_param->mesh->get_verts(), mesh_param->mesh->get_faces());
    
    const Eigen::Matrix<double, Eigen::Dynamic, 3>& e1 = std::get<0>(edges);
    const Eigen::Matrix<double, Eigen::Dynamic, 3>& e2 = std::get<1>(edges);
    const Eigen::Matrix<double, Eigen::Dynamic, 3>& e3 = std::get<2>(edges);

    const Eigen::Matrix<int, Eigen::Dynamic, 2>& e1_vertex_map = mesh_param->e1_vertex_map;
    const Eigen::Matrix<int, Eigen::Dynamic, 2>& e2_vertex_map = mesh_param->e2_vertex_map;
    const Eigen::Matrix<int, Eigen::Dynamic, 2>& e3_vertex_map = mesh_param->e3_vertex_map;
    
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

// ************************* HELPER FUNCTIONS ******************************
std::tuple<Eigen::Matrix<double, Eigen::Dynamic, 3>, 
           Eigen::Matrix<double, Eigen::Dynamic, 3>,
           Eigen::Matrix<double, Eigen::Dynamic, 3> > mesh_edges(const Eigen::Ref<const Eigen::MatrixXd>& V, 
                                                                 const Eigen::Ref<const Eigen::MatrixXi>& F) {

    
    Eigen::Matrix<double, Eigen::Dynamic, 3> V1, V2, V3;
    igl::slice(V, F.col(0), (Eigen::Vector3i() << 0, 1, 2).finished(), V1);
    igl::slice(V, F.col(1), (Eigen::Vector3i() << 0, 1, 2).finished(), V2);
    igl::slice(V, F.col(2), (Eigen::Vector3i() << 0, 1, 2).finished(), V3);

    const Eigen::Matrix<double, Eigen::Dynamic, 3> e1 = V2 - V1;
    const Eigen::Matrix<double, Eigen::Dynamic, 3> e2 = V3 - V2;
    const Eigen::Matrix<double, Eigen::Dynamic, 3> e3 = V1 - V3;

    return std::make_tuple(e1, e2, e3);
}
