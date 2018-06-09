#include "stats.hpp"
#include "mesh.hpp"
#include "cgal_types.hpp"
#include "potential.hpp"
#include "reconstruct.hpp"

#include <iostream>
#include <memory>

namespace Stats {

    void print_surface_mesh_vertices(std::shared_ptr<MeshData> mesh) {
        for (Vertex_index vd : mesh->vertices() ) {
            std::cout << vd  << " " << mesh->get_vertex(vd )<< std::endl;
        }
    }

    void surface_mesh_stats(std::shared_ptr<MeshData> mesh) {
        std::cout << "#Vertices : " << mesh->number_of_vertices() << std::endl;
        std::cout << "#Faces: " << mesh->number_of_faces() << std::endl;
    }
} // end stats namespace

namespace PolyVolume {
    double volume(const Eigen::Ref<const Eigen::MatrixXd> &v,
                  const Eigen::Ref<const Eigen::MatrixXi> &f) {

        double volume(0);
        int a, b, c; 

        Eigen::Matrix<double, 4, 4> tetrahedron_matrix;

        // loop over all faces
        #pragma omp parallel for reduction(+: volume) private(a, b, c, tetrahedron_matrix) shared(f, v)
        for (int ii = 0; ii < f.rows(); ++ii) {
            a = f.row(ii)(0);
            b = f.row(ii)(1);
            c = f.row(ii)(2);
            
            tetrahedron_matrix.row(0) << v(a, 0), v(a, 1), v(a, 2), 1;
            tetrahedron_matrix.row(1) << v(b, 0), v(b, 1), v(b, 2), 1;
            tetrahedron_matrix.row(2) << v(c, 0), v(c, 1), v(c, 2), 1;
            tetrahedron_matrix.row(3) << 0, 0, 0, 1;

            volume = volume + tetrahedron_matrix.determinant();
        }
        return 1.0 / 6.0 * volume;
    }

    double volume(std::shared_ptr<const MeshData> meshdata_ptr) {
        double volume(0);

        // loop over faces
        for (Face_index fd : meshdata_ptr->faces()) {
            Eigen::Matrix<double, 4, 4> tetrahedron_matrix;

            // get vertices of teh face
            std::size_t row = 0;
            for (Vertex_index vd : vertices_around_face(meshdata_ptr->surface_mesh.halfedge(fd), 
                        meshdata_ptr->surface_mesh)){
                Eigen::Vector3d vec = meshdata_ptr->get_vertex(vd);
                tetrahedron_matrix.row(row) << vec(0), vec(1), vec(2), 1;
                ++row;
            } 
            tetrahedron_matrix.row(3) << 0, 0, 0, 1;

            volume = volume+tetrahedron_matrix.determinant();
        }
        return 1.0/6.0 * volume;
    }
    
    double volume(std::shared_ptr<const Asteroid> ast_ptr) {
        return volume(ast_ptr->get_verts(), ast_ptr->get_faces());
    }

    double volume(std::shared_ptr<const MeshParam> meshparam_ptr) {
        return volume(meshparam_ptr->get_verts(), meshparam_ptr->get_faces());
    }
    
    double volume(std::shared_ptr<const ReconstructMesh> rmesh_ptr) {
        return volume(rmesh_ptr->get_verts(), rmesh_ptr->get_faces());
    }
} // End PolyVolume namespace
