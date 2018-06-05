#include "stats.hpp"
#include "mesh.hpp"
#include "cgal_types.hpp"
#include "potential.hpp"
#include "reconstruct.hpp"

#include <iostream>
#include <memory>

void print_polyhedron_vertices(std::shared_ptr<MeshData> mesh) {
    std::copy (mesh->polyhedron.points_begin(), mesh->polyhedron.points_end(), std::ostream_iterator<CGAL::Simple_cartesian<double>::Point_3>(std::cout, "\n")); 
}

void print_surface_mesh_vertices(std::shared_ptr<MeshData> mesh) {
    for (auto ii = mesh->vertex_descriptor.begin(); ii != mesh->vertex_descriptor.end(); ++ii) {
        std::cout << *ii << " " << mesh->surface_mesh.point(*ii) << std::endl;
    }
}

void surface_mesh_stats(std::shared_ptr<MeshData> mesh) {
    
    std::cout << "#Vertices : " << mesh->surface_mesh.number_of_vertices() << std::endl;
    std::cout << "#Faces: " << mesh->surface_mesh.number_of_faces() << std::endl;
}

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
        return volume(meshdata_ptr->get_verts(), meshdata_ptr->get_faces());
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
