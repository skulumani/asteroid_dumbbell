/**
    Example using reconstruction of a mesh

    @author Shankar Kulumani
    @version 11 April 2018
*/
#include "reconstruct.hpp"
#include "mesh.hpp"
#include "input_parser.hpp"
#include "loader.hpp"
#include "geodesic.hpp"

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
        // reconstruct using a point
        ReconstructMesh reconstruct_mesh(mesh);

        Eigen::Vector3d pt(3);
        pt << 5, 0, 0;
        double max_angle(1);
        
        reconstruct_mesh.update(pt, max_angle);
        
    }

    return 0;
}


