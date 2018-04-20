#include "loader.hpp"
#include "mesh.hpp"
#include "reconstruct.hpp"
#include "geodesic.hpp"
#include "surface_mesher.hpp"
#include "cgal.hpp"
#include "polyhedron.hpp"
#include "lidar.hpp"
#include "controller.hpp"

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
    double dist = 5;
    int num_steps = 3;
    sensor.dist(5);

    // place satellite in a specific location and define view axis
    Eigen::RowVector3d pos(3);
    Eigen::Matrix<double, 3, 3> R(3, 3);
    pos << 1.5, 0, 0;
    R = Eigen::MatrixXd::Identity(3, 3); 
    
    // targets to be updated in the loop
    Eigen::Matrix<double, Eigen::Dynamic, 3> targets(num_steps * num_steps, 3);
    Eigen::Matrix<double, Eigen::Dynamic, 3> intersections(num_steps * num_steps, 3);

    // LOOP HERE
    for (int ii = 0; ii < 1; ++ii) {

        // compute targets for use in caster (point at the asteroid origin)
        targets = sensor.define_targets(pos, R, dist);    

        // perform raycasting on the true mesh (true_asteroid pointer)
        intersections = caster.castarray(pos, targets);

        // use this measurement to update rmesh (inside holds teh mesh estimate)
        // use the updated weight to find a new positoin to move to
        // update the state ptr with the newly calculated state
        // save the data (raycast interseciton, position of sat, ast estimate, targets) to hdf5
        // LOOP
    }
    std::cout << intersections << std::endl;
    
    AttitudeController att_control;
    /* att_control.body_fixed_pointing_attitude(0.0, state); */
    return 0;
}
