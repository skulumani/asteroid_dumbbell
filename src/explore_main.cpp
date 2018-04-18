#include "loader.hpp"
#include "mesh.hpp"
#include "reconstruct.hpp"
#include "geodesic.hpp"
#include "surface_mesher.hpp"
#include "cgal.hpp"
#include "polyhedron.hpp"
#include "lidar.hpp"

#include "input_parser.hpp"

#include <memory>
#include <iostream>

int main(int argc, char* argv[])
{
    // CONSTANTS
    // semi major axes of castalia
    Eigen::Vector3d axes(3);
    axes << 1.6130 / 2.0, 0.9810 / 2.0, 0.8260 / 2.0;
    double min_angle(10), max_radius(0.03), max_distance(0.5);

    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Kinematic only exploration with asteroid reconstruction" << std::endl;

    }
    const std::string input_file = input.get_command_option("-i");
    if (input_file.empty()) {
        std::cout << "You need to input file name!" << std::endl;
        std::cout << "explore -i asteroid.obj" << std::endl;
        return 1;
    }

    // initialize all the objects
    std::shared_ptr<MeshData> true_asteroid;
    true_asteroid = Loader::load(input_file);
     
    RayCaster caster(true_asteroid);
    SurfMesh ellipsoid(axes(0), axes(1), axes(2),
            min_angle, max_radius, max_distance);
    ReconstructMesh rmesh(ellipsoid.verts(), ellipsoid.faces());

    Lidar sensor;

    // place satellite in a specific location and define view axis
    // define the initial position of the sensor
    // LOOP HERE
    // compute targets for use in caster (point at the asteroid origin)
    // perform raycasting on the true mesh (true_asteroid pointer)
    // use this measurement to update rmesh (inside holds teh mesh estimate)
    // use the updated weight to find a new positoin to move to
    // save the data (raycast interseciton, position of sat, ast estimate, targets) to hdf5
    // LOOP
    return 0;
}
