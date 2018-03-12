#include "stats.hpp"
/* #include "mesh.hpp" */

#include <CGAL/Simple_cartesian.h>

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
