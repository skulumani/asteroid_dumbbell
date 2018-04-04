#include "loader.hpp"
#include "mesh.hpp"
#include "cgal.hpp"
#include "polyhedron.hpp"

#include "input_parser.hpp"

#include "stats.hpp"

#include <memory>

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
    if (!input_file.empty()) {
        mesh = Loader::load(input_file);
        // lets try and build a surface mesh now
        Eigen::MatrixXd vertices = mesh->get_verts();
        Eigen::MatrixXi faces = mesh->get_faces();

        // update the mesh with new data
        mesh->update_mesh(vertices, faces);

        surface_mesh_stats(mesh);
        /* print_surface_mesh_vertices(mesh); */

        Eigen::Vector3d psource, ptarget;
        psource << 2, 0, 0;
        ptarget << 0, 0, 0;

        // instantiate the raycaster object
        RayCaster caster(mesh);
        Eigen::Vector3d intersection;
        int int_flag = caster.castray(psource, ptarget, intersection);
        if (int_flag == 0) {
            std::cout << "Intersection point: " << intersection << std::endl;
        }

        // Distance to mesh object
        MeshDistance mesh_dist(mesh);

        mesh_dist.k_nearest_neighbor(psource, 5);

        // compute the volume of the mesh
        double vol;
        vol = polyhedron_volume(mesh->vertices, mesh->faces);
        std::cout << "Volume: " << vol << std::endl;

    }

    /* std::cout << "Vertices: \n" << mesh->vertices << std::endl; */
    /* std::cout << "Faces: \n" << mesh->faces << std::endl; */

    /* print_polyhedron_vertices(mesh); */
    
    return 0;
}
