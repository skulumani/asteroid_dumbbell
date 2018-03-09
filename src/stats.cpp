#include "stats.hpp"
/* #include "mesh.hpp" */

#include <CGAL/Simple_cartesian.h>

#include <iostream>
#include <memory>

void print_polyhedron_vertices(std::shared_ptr<MeshData> mesh) {
    std::copy (mesh->polyhedron.points_begin(), mesh->polyhedron.points_end(), std::ostream_iterator<CGAL::Simple_cartesian<double>::Point_3>(std::cout, "\n")); 
}
