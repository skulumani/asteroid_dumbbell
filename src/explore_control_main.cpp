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
#include "potential.hpp"

#include "hdf5.hpp"

#include "input_parser.hpp"

#include <Eigen/Dense>
#include "H5Cpp.h"

#include <memory>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{

    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Kinematic only exploration with asteroid reconstruction" << std::endl;
    }

    const std::string input_file = "./data/shape_model/CASTALIA/castalia.obj";
    
    const std::string output_file = input.get_command_option("-o");
    if (output_file.empty()) {
        std::cout << "You need an output file name!" << std::endl;
        std::cout << "explore -o data.hdf5" << std::endl;
        return 1;
    }
    // CONSTANTS
    double min_angle(10), max_radius(0.03), max_distance(0.5);
    double surf_area(0.01);

    // initialize true objects
    std::shared_ptr<MeshData> true_meshdata_ptr = Loader::load(input_file);
    std::shared_ptr<Asteroid> true_ast_ptr = std::make_shared<Asteroid>("castalia", true_meshdata_ptr);
    
    // initialize estimate objects
    SurfMesh ellipsoid(true_ast_ptr->get_axes()[0], true_ast_ptr->get_axes()[1], true_ast_ptr->get_axes()[2],
                       min_angle, max_radius, max_distance);
    std::shared_ptr<MeshData> est_meshdata_ptr = std::make_shared<MeshData>(ellipsoid.get_verts(), ellipsoid.get_faces());
    std::shared_ptr<ReconstructMesh> est_rmesh_ptr = std::make_shared<ReconstructMesh>(est_meshdata_ptr);
    std::shared_ptr<Asteroid> est_ast_ptr = std::make_shared<Asteroid>("castalia", est_rmesh_ptr);
    
    // other objects
    RayCaster caster(true_meshdata_ptr);
    Controller controller(est_meshdata_ptr);

    Lidar sensor;
    // initialize all the objects
    double max_angle( pow(surf_area / (true_ast_ptr->get_axes()[0] * true_ast_ptr->get_axes()[0]), 0.5));
     
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
    // state is now in the inertial frame
    controller.explore_asteroid(0, state_ptr, est_rmesh_ptr, est_ast_ptr);
    new_state_ptr = controller.get_desired_state();
    state_ptr->update_state(new_state_ptr);
    
    // targets to be updated in the loop
    Eigen::Matrix<double, 1, Eigen::Dynamic> target(1, 3);
    Eigen::Matrix<double, 1, Eigen::Dynamic> intersection(1, 3);
    
    // save initial data to the HDF5 file
    hf->write("truth_vertex", true_ast_ptr->get_verts());
    hf->write("truth_faces", true_ast_ptr->get_faces());
    
    hf->write("initial_vertex", est_rmesh_ptr->get_verts());
    hf->write("initial_faces", est_rmesh_ptr->get_faces());
    hf->write("initial_weight", est_rmesh_ptr->get_weights());

    hf->write("initial_state", state_ptr->get_state()); 

    // LOOP HERE
    for (int ii = 0; ii < 1000; ++ii) {
        
        // compute targets for use in caster (point at the asteroid origin)
        target = sensor.define_target(state_ptr->get_pos(), state_ptr->get_att(), dist);    

        // perform raycasting on the true mesh (true_asteroid pointer)
        intersection = caster.castray(state_ptr->get_pos(), target);

        // use this measurement to update rmesh (inside holds teh mesh estimate)
        est_rmesh_ptr->single_update(intersection, max_angle);
        // use the updated weight to find a new positoin to move to
        controller.explore_asteroid(0, state_ptr, est_rmesh_ptr, true_ast_ptr);
        new_state_ptr = controller.get_desired_state();
        // update the state ptr with the newly calculated state
        state_ptr->update_state(new_state_ptr);
        // save the data (raycast interseciton, position of sat, ast estimate, targets) to hdf5
        reconstructed_vertex_group.write(std::to_string(ii), est_rmesh_ptr->get_verts());
        reconstructed_weight_group.write(std::to_string(ii), est_rmesh_ptr->get_weights());
        state_group.write(std::to_string(ii), state_ptr->get_state());
        targets_group.write(std::to_string(ii), target);
        intersections_group.write(std::to_string(ii), intersection);

    }
    
    /* /1* hf.close(); *1/ */
    return 0;
}
