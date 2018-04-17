#include "loader.hpp"
#include "mesh.hpp"
#include "reconstruct.hpp"
#include "geodesic.hpp"
#include "input_parser.hpp"

#include <memory>
#include <iostream>

int main(int argc, char* argv[])
{
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Kinematic only exploration with asteroid reconstruction" << std::endl;

    }
    const std::string input_file input.get_command_option("-i");
    if (input_file.empty()) {
        std::cout << "You need to input file name!" << std::endl;
        std::cout << "explore -i asteroid.obj" << st d::endl;
        return 1;
    }
    
    // initialize all the objects
    std::shared_ptr<MeshData> true_asteroid;
    true_asteroid = Loader::load(input_file);
    
    RayCaster caster(true_asteroid);
    SurfMesh ellipsoid(a, b, c, min_angle, max_radius, max_distance);

    // define the initial weight of all the vertices
    ReconstructMesh rmesh(ellipsoid.verts(), ellipsoid.faces(), weights);
    // load the asteroid mesh and an estimate of the asteroid
    // place satellite in a specific location and define view axis
    // compute the intersections using the raycaster
    // reconstruct the mesh with the estimate
    // compute a new position
    // loop over new position
    return 0;
}
