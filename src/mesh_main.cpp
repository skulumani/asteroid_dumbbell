#include "loader.hpp"
#include "mesh.hpp"
#include "cgal.hpp"
#include "polyhedron.hpp"
#include "stats.hpp"

#include "input_parser.hpp"

#include <CGAL/Polygon_mesh_processing/refine.h>

#include <memory>
#include <iostream>
#include <fstream>

// try to write a function to  find distance from mesh to point using AABB
//
int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage mesh -i input_file.obj" << std::endl;
    }

    const std::string input_file = input.get_command_option("-i");
    std::shared_ptr<MeshData> mesh;

    // create a mesh object
    if (input_file.empty()) {
        std::cout << "No input file!!! mesh -h" << std::endl;
        return 1;
    }

    mesh = Loader::load(input_file);
    // lets try and build a surface mesh now
    Eigen::MatrixXd v_eigen = mesh->get_verts();
    Eigen::MatrixXi f_eigen = mesh->get_faces();

    // update the mesh with new data
    mesh->update_mesh(v_eigen, f_eigen);

    surface_mesh_stats(mesh);

    /* print_surface_mesh_vertices(mesh); */

    Eigen::Vector3d psource, ptarget;
    psource << 2, 0, 0;
    ptarget << 0, 0, 0;

    // instantiate the raycaster object
    RayCaster caster(mesh);
    Eigen::Vector3d intersection;
    intersection = caster.castray(psource, ptarget);
    std::cout << "Intersection point: " << intersection << std::endl;
    // compute minimum distance to mesh
    double min_dist = caster.minimum_distance(psource);
    std::cout << "Minimum distance: " << min_dist << std::endl;

    // Distance to mesh object
    MeshDistance mesh_dist(mesh);

    mesh_dist.k_nearest_neighbor(psource, 5);

    // compute the volume of the mesh
    double vol;
    vol = PolyVolume::volume(mesh->vertices, mesh->faces);
    std::cout << "Volume: " << vol << std::endl;
    
    // try to refine a portion of the mesh
    // need face descriptors for faces to refine
    Face_index f_index = mesh->face_descriptor[0];
    std::vector<Face_index> faces_to_refine;
    faces_to_refine.push_back(mesh->face_descriptor[0]);
    faces_to_refine.push_back(mesh->face_descriptor[1]);
    // make std;:vectors to store the descriptors to the new faces/vertices
    std::vector<Face_index> new_faces;
    std::vector<Vertex_index> new_vertices;
    CGAL::Polygon_mesh_processing::refine(
            mesh->surface_mesh,
            faces_to_refine,
            std::back_inserter(new_faces),
            std::back_inserter(new_vertices),
            CGAL::Polygon_mesh_processing::parameters::density_control_factor(100.0));

    // save to a different obj for visualizaiton in python
    std::ofstream refined_off("/tmp/cube_refined.off");
    refined_off << mesh->surface_mesh;
    refined_off.close();
    std::cout << "Refinement added " << new_vertices.size() << " vertices." << std::endl;
    /* std::cout << "Vertices: \n" << mesh->vertices << std::endl; */
    /* std::cout << "Faces: \n" << mesh->faces << std::endl; */

    /* print_polyhedron_vertices(mesh); */

    return 0;
}
