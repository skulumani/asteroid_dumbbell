/**
    Example using reconstruction of a mesh

    @author Shankar Kulumani
    @version 11 April 2018
*/
#include "reconstruct.hpp"
#include "mesh.hpp"
#include "input_parser.hpp"
#include "loader.hpp"

int main(int argc, char* argv[])
{
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage mesh -i input_file.obj" << std::endl;
    }

    const std::string input_file = input.get_command_option("-i");
    std::shared_ptr<MeshData> mesh;

    if (!input_file.empty()) {
        // create the mesh
        mesh = Loader::load(input_file);
    }

    // reconstruct using a point
    ReconstructMesh reconstruct_mesh(mesh);

    Eigen::Vector3d pt(1, 1, 1);
    double max_angle(1);

    std::cout << reconstruct_mesh.get_verts() << std::endl << std::endl;

    reconstruct_mesh.update(pt, max_angle);

    return 0;
}


