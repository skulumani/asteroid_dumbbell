#include "cgal.hpp"

#include <Eigen/Dense>

#include <cmath>

// Raycaster class
RayCaster::RayCaster(std::shared_ptr<MeshData> mesh_in) {
    // assign copy of pointer to object instance
    this->mesh = mesh_in;
    this->tree.insert(faces(this->mesh->polyhedron).first,
            faces(this->mesh->polyhedron).second,
            this->mesh->polyhedron);
    tree.accelerate_distance_queries();
}

void RayCaster::update_mesh(std::shared_ptr<MeshData> mesh_in) {
    this->mesh = mesh_in;
    this->tree.clear();
    this->tree.insert(faces(this->mesh->polyhedron).first,
            faces(this->mesh->polyhedron).second,
            this->mesh->polyhedron);
    tree.accelerate_distance_queries();
}

Eigen::Matrix<double, 1, 3> RayCaster::castray(const Eigen::Ref<const Eigen::Vector3d>& psource, const Eigen::Ref<const Eigen::Vector3d>& ptarget) {
    // TODO Also look at closest_point_and_primitive
    // create a Point object
    Point a(psource(0),psource(1),psource(2));
    Point b(ptarget(0), ptarget(1), ptarget(2));
    Ray ray_query(a, b);
    Eigen::Matrix<double, 1, 3> pint(3);

    // May need to add a skip function here https://doc.cgal.org/latest/AABB_tree/AABB_tree_2AABB_ray_shooting_example_8cpp-example.html
    Ray_intersection intersection = this->tree.first_intersection(ray_query);
    if (intersection) {
        // get intersection object
        const Point* p = boost::get<Point>(&(intersection->first));
        if (p) {
            // output from function
            pint << CGAL::to_double(p->x()), CGAL::to_double(p->y()), CGAL::to_double(p->y());
        } else {
            return pint.setZero();
        }

    } else {
        return pint.setZero();
    }

    return pint;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> RayCaster::castarray(const Eigen::Ref<const Eigen::Vector3d> &psource,
                                                              const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 3> > &targets) {
    
    int num_targets = targets.rows();

    Eigen::Matrix<double, Eigen::Dynamic, 3> all_intersections(num_targets, 3), intersection(1, 3);

    for (int ii = 0; ii < num_targets; ++ii) {
        intersection = this->castray(psource, targets.row(ii));
        all_intersections.row(ii) = intersection;
    }
    return intersection;
}

// TODO Modify this to compute distance instead of doing raycasting
double RayCaster::minimum_distance(const Eigen::Ref<const Eigen::Vector3d> &pt) {

    // create a Point object
    Point a(pt(0), pt(1), pt(2));
    
    return sqrt(CGAL::to_double(tree.squared_distance(a)));
 }

void RayCaster::minimum_primitive(const Eigen::Ref<const Eigen::Vector3d> &pt) {

}

// MeshDistance class
// Raycaster class
MeshDistance::MeshDistance(std::shared_ptr<MeshData> mesh_in) {
    // assign copy of pointer to object instance
    this->mesh = mesh_in;

    this->vppmap = get(CGAL::vertex_point, this->mesh->surface_mesh);

    // TODO Figure out how to save KD tree to the class
    
}

void MeshDistance::update_mesh(std::shared_ptr<MeshData> mesh_in) {
    this->mesh = mesh_in;
    this->vppmap = get(CGAL::vertex_point, this->mesh->surface_mesh);

}

int MeshDistance::k_nearest_neighbor(const Eigen::Ref<const Eigen::Vector3d>& pt, const int &K) {
    // TODO Move this property map into the mesh data class itself 
    Vertex_point_pmap vppmap = get(CGAL::vertex_point, this->mesh->surface_mesh);

    // TODO Figure out how to build this only once insert number of data points in the tree
    KD_Tree tree(vertices(this->mesh->surface_mesh).begin(),
            vertices(this->mesh->surface_mesh).end(),
            Splitter(),
            KD_Traits(vppmap));
    Point query(pt(0),  pt(1), pt(2));
    Distance tr_dist(vppmap);

    // Save the nearest neighbors into an Eigen array
    K_neighbor_search search(tree, query, K, 0, true, tr_dist);
    for (K_neighbor_search::iterator it = search.begin(); it != search.end(); it++) {
        std::cout << "Vertex " << it->first << " : " << vppmap[it->first] << " at distance " 
            << tr_dist.inverse_of_transformed_distance(it->second) << std::endl;
    }
    return 0;
}

