#ifndef POTENTIAL_H
#define POTENTIAL_H

#include <Eigen/Dense>

#include <vector>
#include <tuple>

// Make a mesh parameter class to hold all the data computed from polyhedron parameters

class MeshParam {
    private:
        void polyhedron_parameters( void );

    public:
        MeshParam( void ) {};
        virtual ~MeshParam( void ) {};

        MeshParam(const Eigen::Ref<const Eigen::Array<double, Eigen::Dynamic, 3> >& V_in, 
                const Eigen::Ref<const Eigen::Array<int, Eigen::Dynamic, 3> >& F_in);
    

        // define all the member variables
        std::size_t num_v, num_f, num_e;

        Eigen::Array<double, Eigen::Dynamic, 3> V;
        Eigen::Array<int, Eigen::Dynamic, 3> F;

        Eigen::Matrix<int, Eigen::Dynamic, 1> Fa, Fb, Fc;
        Eigen::Matrix<double, Eigen::Dynamic, 3> V1, V2, V3;
        Eigen::Matrix<double, Eigen::Dynamic, 3> e1, e2, e3;

        Eigen::Matrix<int, Eigen::Dynamic, 2> e1_vertex_map;
        Eigen::Matrix<int, Eigen::Dynamic, 2> e2_vertex_map;
        Eigen::Matrix<int, Eigen::Dynamic, 2> e3_vertex_map;

        Eigen::MatrixXi unique_index, e_vertex_map;

        Eigen::Matrix<double, Eigen::Dynamic, 3> normal_face,
                                                 e1_normal,
                                                 e2_normal,
                                                 e3_normal,
                                                 center_face;
        std::tuple<Eigen::Matrix<int, Eigen::Dynamic, 2>,
                   Eigen::Matrix<int, Eigen::Dynamic, 2>,
                   Eigen::Matrix<int, Eigen::Dynamic, 2> > edge_vertex_map;

        std::vector<std::vector<int> > vf_map;

        Eigen::VectorXi e1_ind1b, e1_ind2b, e1_ind3b,
                        e2_ind1b, e2_ind2b, e2_ind3b,
                        e3_ind1b, e3_ind2b, e3_ind3b;

        std::tuple<Eigen::MatrixXi, Eigen::MatrixXi, Eigen::MatrixXi> edge_face_map;
        Eigen::MatrixXi e1_face_map,
                        e2_face_map,
                        e3_face_map;

};

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


std::vector<std::vector<int> > vertex_face_map(const Eigen::Ref<const Eigen::MatrixXd> &V,
                                                const Eigen::Ref<const Eigen::MatrixXi> &F);

std::tuple<Eigen::VectorXi, Eigen::VectorXi> search_index(const Eigen::Ref<const Eigen::VectorXi>& a, const Eigen::Ref<const Eigen::VectorXi>& b);

Eigen::VectorXi vertex_map_search(const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& a_map,
        const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& b_map);
#endif
