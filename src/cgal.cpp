#include "cgal.hpp"

// TODO Modify this to compute distance instead of doing raycasting
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
    
    std::cout << tree.number_of_intersected_primitives(segment_query) << " intersection(s)" << std::endl;


    Segment_intersection intersection = tree.any_intersection(segment_query);
    if(intersection) {
        // get intersection object
        const Point* p = boost::get<Point>(&(intersection->first));
        if(p) {
            std::cout << "intersection object is a point " << *p << std::endl;
        }
    }
}

// Raycaster class
RayCaster::RayCaster(std::shared_ptr<MeshData> mesh) {
    // assign copy of pointer to object instance
    this->mesh = mesh;
    this->tree.insert(faces(this->mesh->polyhedron).first,
            faces(this->mesh->polyhedron).second,
            this->mesh->polyhedron);
}

void RayCaster::update_mesh(std::shared_ptr<MeshData> mesh) {
    this->mesh = mesh;
    this->tree.clear();
    this->tree.insert(faces(this->mesh->polyhedron).first,
            faces(this->mesh->polyhedron).second,
            this->mesh->polyhedron);
}

void RayCaster::castray(Eigen::Vector3d& psource, Eigen::Vector3d& ptarget) {
    // create a Point object
    Point a(psource(0),psource(1),psource(2));
    Point b(ptarget(0), ptarget(1), ptarget(2));
    Segment segment_query(a, b);

    // tests intersections with segment query
    if(this->tree.do_intersect(segment_query)) {
        std::cout << "intersection(s)" << std::endl;
    } else {
        std::cout << "no intersection" << std::endl;
    }

    std::cout << this->tree.number_of_intersected_primitives(segment_query) << " intersection(s)" << std::endl;


    Segment_intersection intersection = this->tree.any_intersection(segment_query);
    if(intersection) {
        // get intersection object
        const Point* p = boost::get<Point>(&(intersection->first));
        if(p) {
            std::cout << "intersection object is a point " << *p << std::endl;
        }
    }
}
