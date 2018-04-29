#ifndef POTENTIAL_H
#define POTENTIAL_H

#include <Eigen/Dense>

#include <vector>
#include <tuple>

// declare some shit
void face_contribution_loop(Eigen::Vector3d r_v,  Eigen::MatrixXd V, Eigen::MatrixXi F, 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> F_face, Eigen::Matrix<double, Eigen::Dynamic, 1> w_face);


int laplacian_factor(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& r_v,
                     const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 1> >& Fa,
                     const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 1> >& Fb,
                     const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 1> >& Fc,
                     Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > w_face);

int edge_factor(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& r_v, 
                const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& e1,
                const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& e2,
                const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& e3,
                const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 2> >& e1_vertex_map,
                const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 2> >& e2_vertex_map,
                const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 2> >& e3_vertex_map,
                Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > L1_edge,
                Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > L2_edge,
                Eigen::Ref<Eigen::Array<double, Eigen::Dynamic, 1> > L3_edge);

void polyhedron_parameters(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& V,
        const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 3> >& F);


std::vector<std::vector<int> > vertex_face_map(const Eigen::Ref<const Eigen::MatrixXd> &V,
                                                const Eigen::Ref<const Eigen::MatrixXi> &F);

std::tuple<Eigen::VectorXi, Eigen::VectorXi> search_index(const Eigen::Ref<const Eigen::VectorXi>& a, const Eigen::Ref<const Eigen::VectorXi>& b);

Eigen::VectorXi vertex_map_search(const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& a_map,
        const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& b_map);
#endif
