#include "loader.hpp"
#include "mesh.hpp"

#include "input_parser.hpp"

#include "stats.hpp"

#include <memory>


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

    }

    /* std::cout << "Vertices: \n" << mesh->vertices << std::endl; */
    /* std::cout << "Faces: \n" << mesh->faces << std::endl; */

    print_polyhedron_vertices(mesh);
    
    // lets try and build a surface mesh now
    
    surface_mesh_stats(mesh);
    print_surface_mesh_vertices(mesh);
    
    return 0;
}
