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
    // load the asteroid mesh and an estimate of the asteroid
    // place satellite in a specific location and define view axis
    // compute the intersections using the raycaster
    // reconstruct the mesh with the estimate
    // compute a new position
    // loop over new position
    return 0;
}
