/**
    Polyhedron Potential model for an asteroid

    @author Shankar Kulumani
    @version 3 May 2018
*/
#ifndef POTENTIAL_H
#define POTENTIAL_H

#include "mesh.hpp"
#include "reconstruct.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <vector>
#include <tuple>
#include <memory>
#include <string>

/** @class MeshParam

    @brief Asteroid parameters required for the polyhedron potential function
    
    This class computes and contains all the parameters required by the 
    polyhedron potential function. It precomputes lots of data that is then used
    in Asteroid class.

    @author Shankar Kulumani
    @version 3 May 2018
*/
class MeshParam {
    private:
        /** @fn void polyhedron_parameters( void )
                
            Compute all the polyhedron parameters

            @author Shankar Kulumani
            @version 3 May 2018
        */
        void polyhedron_parameters( void );

        /** @fn void face_dyad( void )
                
            Compute the face dyad F_face

            @author Shankar Kulumani
            @version 3 May 2018
        */
        void face_dyad( void );

        /** @fn void edge_dyad( void )
                
            Compute the edge dyad E_edge

            @author Shankar Kulumani
            @version 3 May 2018
        */
        void edge_dyad( void );

    public:
        MeshParam( void ) {};
        virtual ~MeshParam( void ) {};

        MeshParam(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& V_in, 
                const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 3> >& F_in);
        
        MeshParam( std::shared_ptr<MeshData> mesh_in);
		
		std::shared_ptr<MeshData> mesh;

        // define all the member variables
        std::size_t num_v, num_f, num_e;
        
        // Try to get rid of and use indexing instead
        Eigen::Matrix<int, Eigen::Dynamic, 1> Fa, Fb, Fc;
        Eigen::Matrix<double, Eigen::Dynamic, 3> e1, e2, e3;
        
        // required
        Eigen::Matrix<int, Eigen::Dynamic, 2> e1_vertex_map;
        Eigen::Matrix<int, Eigen::Dynamic, 2> e2_vertex_map;
        Eigen::Matrix<int, Eigen::Dynamic, 2> e3_vertex_map;
        
        // required unique_index, e_vertex_map
        Eigen::MatrixXi unique_index; /**< Unique indices of e_vertex_map_sorted */
        Eigen::MatrixXi e_vertex_map, e_vertex_map_stacked, e_vertex_map_sorted;

        Eigen::Matrix<double, Eigen::Dynamic, 3> normal_face,
                                                 e1_normal,
                                                 e2_normal,
                                                 e3_normal,
                                                 center_face;
        
        // required
        std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3> > > F_face;
        std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3> >  > E1_edge, E2_edge, E3_edge;

};

class Asteroid {
    private: 
        // member variables to hold the potential
        const double G = 6.673e-20; /**< Gravitational constant - km^3/kg/sec^2 */
        double sigma; /**< Density - kg/km^3 */
        std::string name; /**< Asteroid name - string */
        double M; /**< Mass - kg */
        Eigen::Vector3d axes; /**< Semi-major axes - km */
        double omega; /**< Rotation rate - rad/sec */

        std::shared_ptr<MeshParam> mesh_param;

        double U; 
        Eigen::Vector3d U_grad;
        Eigen::Matrix3d U_grad_mat;
        double Ulaplace;

        void init_asteroid( void );

    public:
        Asteroid ( void ) {};
        virtual ~Asteroid ( void ) {};
        
        Asteroid(const std::string& name_in, MeshParam& mesh_param);
        Asteroid(const std::string& name_in, std::shared_ptr<MeshParam> mesh_param);

        Asteroid(const std::string& name_in, std::shared_ptr<ReconstructMesh> rmesh_in);

        Asteroid(const std::string& name_in, std::shared_ptr<MeshData> mesh_in);

        // Functions to compute the potential
        Eigen::Matrix<double, Eigen::Dynamic, 1> laplacian_factor(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v);

        std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> edge_factor(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v);

        std::tuple<double, Eigen::Matrix<double, 3, 1>, Eigen::Matrix<double, 3, 3> > face_contribution(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v,
                const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1> >& w_face);

        std::tuple<double, Eigen::Matrix<double, 3, 1>, Eigen::Matrix<double, 3, 3> > edge_contribution(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> >& r_v,
                const std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>& L_tuple);

        /** @fn void polyhedron_potential(const Eigen::Ref<const Eigen::Vector3d>& state)
                
            Compute the polyhedron potential at the given state

            @param state Eigen Vector3d defining the state in the asteroid body fixed frame in km
            @returns None

            @author Shankar Kulumani
            @version 3 May 2018
        */
        void polyhedron_potential(const Eigen::Ref<const Eigen::Vector3d>& state);
        
        Eigen::Matrix<double, Eigen::Dynamic, 3> rotate_vertices(const double& time) const;

        Eigen::Matrix<double, 3, 3> rot_ast2int(const double& time);

        // Getters for the potential variables
        double get_potential( void ) { return U; }
        Eigen::Vector3d get_acceleration( void ) { return U_grad; }
        Eigen::Matrix3d get_gradient_mat( void ) { return U_grad_mat; }
        double get_laplace( void ) { return Ulaplace; }
        double get_omega( void) const { return omega; }

        // member variables
        Eigen::Vector3d get_axes( void ) const { return axes; }

};
// declare some shit
std::vector<std::vector<int> > vertex_face_map(const Eigen::Ref<const Eigen::MatrixXd> &V,
                                                const Eigen::Ref<const Eigen::MatrixXi> &F);

std::tuple<Eigen::VectorXi, Eigen::VectorXi> search_index(const Eigen::Ref<const Eigen::VectorXi>& a, const Eigen::Ref<const Eigen::VectorXi>& b);

Eigen::VectorXi vertex_map_search(const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& a_map,
        const Eigen::Ref<const Eigen::Matrix<int, Eigen::Dynamic, 2> >& b_map);

#endif
