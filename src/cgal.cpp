#include "cgal.hpp"

void distance_to_polyhedron(Eigen::Vector3d& pt, std::shared_ptr<MeshData> mesh) {
    Tree tree(faces(mesh->polyhedron).first, faces(mesh->polyhedron).second, mesh->polyhedron);
    
    // create a Point object
    Point a(pt(0), pt(1), pt(2));
    Point b(0, 0, 0);
    Segment segment_query(a, b);

    // tests intersections with segment query
    if(tree.do_intersect(segment_query)) {
        std::cout << "intersection(s)" << std::endl;
    } else {
        std::cout << "no intersection" << std::endl;
    }

}
