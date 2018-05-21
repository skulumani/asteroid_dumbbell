#include "loader.hpp"
#include "mesh.hpp"
#include "reconstruct.hpp"
#include "geodesic.hpp"
#include "surface_mesher.hpp"
#include "cgal.hpp"
#include "polyhedron.hpp"
#include "lidar.hpp"
#include "controller.hpp"
#include "state.hpp"
#include "hdf5.hpp"

#include "input_parser.hpp"

#include <Eigen/Dense>
#include "H5Cpp.h"

#include <memory>
#include <iostream>
#include <string>

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
        std::cout << "explore -i asteroid.obj -o data.hdf5" << std::endl;
        return 1;
    }
    
    const std::string output_file = input.get_command_option("-o");
    if (output_file.empty()) {
        std::cout << "You need an output file name!" << std::endl;
        std::cout << "explore -i asteroid.obj -o data.hdf5" << std::endl;
        return 1;
    }

    // initialize all the objects
    double surf_area(0.01);
    double max_angle( pow(surf_area / (axes(0) * axes(0)), 0.5));

    std::shared_ptr<MeshData> true_asteroid;
    true_asteroid = Loader::load(input_file);
     
    RayCaster caster(true_asteroid);
    SurfMesh ellipsoid(axes(0), axes(1), axes(2),
            min_angle, max_radius, max_distance);
    std::shared_ptr<ReconstructMesh> rmesh_ptr = std::make_shared<ReconstructMesh>(ellipsoid.get_verts(), ellipsoid.get_faces());
    Controller controller;

    Lidar sensor;
    double dist = 5;
    int num_steps = 3;
    sensor.dist(dist).num_steps(num_steps);
    
    // Create HDF5 file for saving the data
    std::shared_ptr<HDF5::File> hf = std::make_shared<HDF5::File>(output_file, HDF5::File::Truncate);
    
    HDF5::Group reconstructed_vertex_group(hf.get(), "reconstructed_vertex"),
                reconstructed_weight_group(hf.get(), "reconstructed_weight"),
                state_group(hf.get(), "state"),
                targets_group(hf.get(), "targets"),
                intersections_group(hf.get(), "intersections");

    // place satellite in a specific location and define view axis
    State initial_state, state;
    initial_state.pos((Eigen::RowVector3d() << 1.5, 0, 0).finished())
         .vel((Eigen::RowVector3d() << 0, 0, 0).finished())
         .att((Eigen::MatrixXd::Identity(3, 3)))
         .ang_vel((Eigen::RowVector3d() << 0, 0, 0).finished());
    std::shared_ptr<State> state_ptr = std::make_shared<State>(state);
    std::shared_ptr<State> initial_state_ptr = std::make_shared<State>(initial_state);
    std::shared_ptr<State> new_state_ptr = std::make_shared<State>();

    state_ptr->update_state(initial_state_ptr);
    // modify the initial state to point at the body using the controller
    controller.explore_asteroid(state_ptr, rmesh_ptr);
    new_state_ptr = controller.get_desired_state();
    state_ptr->update_state(new_state_ptr);
    
    // targets to be updated in the loop
    Eigen::Matrix<double, 1, Eigen::Dynamic> target(1, 3);
    Eigen::Matrix<double, 1, Eigen::Dynamic> intersection(1, 3);
    
    // save initial data to the HDF5 file
    hf->write("truth_vertex", true_asteroid->get_verts());
    hf->write("truth_faces", true_asteroid->get_faces());
    
    hf->write("initial_vertex", rmesh_ptr->get_verts());
    hf->write("initial_faces", rmesh_ptr->get_faces());
    hf->write("initial_weight", rmesh_ptr->get_weights());

    hf->write("initial_state", state_ptr->get_state()); 

    // LOOP HERE
    for (int ii = 0; ii < rmesh_ptr->get_verts().rows(); ++ii) {
        
        // compute targets for use in caster (point at the asteroid origin)
        target = sensor.define_target(state_ptr->get_pos(), state_ptr->get_att(), dist);    

        // perform raycasting on the true mesh (true_asteroid pointer)
        intersection = caster.castray(state_ptr->get_pos(), target);

        // use this measurement to update rmesh (inside holds teh mesh estimate)
        rmesh_ptr->single_update(intersection, max_angle);
        // use the updated weight to find a new positoin to move to
        controller.explore_asteroid(state_ptr, rmesh_ptr);
        new_state_ptr = controller.get_desired_state();
        // update the state ptr with the newly calculated state
        state_ptr->update_state(new_state_ptr);
        // save the data (raycast interseciton, position of sat, ast estimate, targets) to hdf5
        reconstructed_vertex_group.write(std::to_string(ii), rmesh_ptr->get_verts());
        reconstructed_weight_group.write(std::to_string(ii), rmesh_ptr->get_weights());
        state_group.write(std::to_string(ii), state_ptr->get_state());
        targets_group.write(std::to_string(ii), target);
        intersections_group.write(std::to_string(ii), intersection);

    }
    
    /* hf.close(); */
    return 0;
}
