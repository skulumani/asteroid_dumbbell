#include "stats.hpp"
#include "mesh.hpp"
#include "cgal_types.hpp"

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
double volume(const Eigen::Ref<const Eigen::MatrixXd> &v, const Eigen::Ref<const Eigen::MatrixXi> &f) {
    
    double volume(0);
    int a, b, c; 

    Eigen::Vector3d v1, v2, v3;

    Eigen::Matrix<double, 4, 4> tetrahedron_matrix;

    // loop over all faces
    for(int ii = 0; ii != f.rows(); ++ii) {
        a = f.row(ii)[0];
        b = f.row(ii)[1];
        c = f.row(ii)[2];
        
        v1 << v.row(a);
        v2 << v.row(b);
        v3 << v.row(c);
        
        tetrahedron_matrix.row(0) << v(a, 0), v(a, 1), v(a, 2), 1;
        tetrahedron_matrix.row(1) << v(b, 0), v(b, 1), v(b, 2), 1;
        tetrahedron_matrix.row(2) << v(c, 0), v(c, 1), v(c, 2), 1;
        tetrahedron_matrix.row(3) << 0, 0, 0, 1;

        volume = volume + tetrahedron_matrix.determinant();
    }
    return 1.0 / 6.0 * volume;
}

}
