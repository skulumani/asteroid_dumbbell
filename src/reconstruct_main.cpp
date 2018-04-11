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

        Eigen::Vector3d pt(1, 1, 1);
        double max_angle(1);


        reconstruct_mesh.update(pt, max_angle);
        reconstruct_mesh.update_meshdata();

        std::cout << reconstruct_mesh.get_verts() << std::endl << std::endl;
        std::cout << mesh->get_verts() << std::endl;
    }

    Eigen::Vector3d s1(3);
    Eigen::Matrix<double, 1, 3> s2(3);
    s1 << 1, 0, 0;
    s2 << 1, 0, 0;
    
    Eigen::VectorXd delta_sigma;
    delta_sigma = central_angle(s1, s2);

    std::cout << delta_sigma << std::endl;


    return 0;
}


