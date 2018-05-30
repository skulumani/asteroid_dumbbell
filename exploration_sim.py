"""Exploration Simulation

Simulation of a spacecraft exploring an asteroid

States are chosen to minimize a cost function tied to the uncertainty of the 
shape. Full SE(3) dynamics and the polyhedron potential model is used

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import pdb
import logging
import os
import tempfile
import argparse
from collections import defaultdict

import h5py
import numpy as np
from scipy import integrate

from lib import asteroid, surface_mesh, cgal, mesh_data, reconstruct
from lib import surface_mesh
from lib import controller as controller_cpp

from dynamics import dumbbell, eoms, controller
from point_cloud import wavefront
from kinematics import attitude
import utilities
from visualization import graphics, animation

compression = 'gzip'
compression_opts = 4

def initialize(hf):
    """Initialize all the things for the simulation

    Output_file : the actual HDF5 file to save the data/parameters to

    """
    logger = logging.getLogger(__name__)
    logger.info('Initialize asteroid and dumbbell objects')

    AbsTol = 1e-9
    RelTol = 1e-9
    
    # true asteroid and dumbbell
    v, f = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj')

    true_ast_meshdata = mesh_data.MeshData(v, f)
    true_ast_meshparam = asteroid.MeshParam(true_ast_meshdata)
    true_ast = asteroid.Asteroid('castalia', true_ast_meshparam)

    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
    
    # estimated asteroid (starting as an ellipse)
    surf_area = 0.01
    max_angle = np.sqrt(surf_area / true_ast.get_axes()[0]**2)
    min_angle = 10
    max_distance = 0.5
    max_radius = 0.03

    ellipsoid = surface_mesh.SurfMesh(true_ast.get_axes()[0], true_ast.get_axes()[1], true_ast.get_axes()[2],
                                      min_angle, max_radius, max_distance)

    est_ast_meshdata = mesh_data.MeshData(ellipsoid.get_verts(), ellipsoid.get_faces())
    est_ast_rmesh = reconstruct.ReconstructMesh(est_ast_meshdata)
    est_ast = asteroid.Asteroid("castalia", est_ast_rmesh)

    # controller functions 
    complete_controller = controller_cpp.Controller()
    
    # lidar object
    lidar = cgal.Lidar()
    lidar = lidar.view_axis(np.array([1, 0, 0]))
    lidar = lidar.up_axis(np.array([0, 0, 1]))
    lidar = lidar.fov(np.deg2rad(np.array([7, 7]))).dist(2).num_steps(3)

    # raycaster from c++
    caster = cgal.RayCaster(true_ast_meshdata)
    
    # save a bunch of parameters to the HDF5 file
    sim_group = hf.create_group("simulation_parameters")
    sim_group['AbsTol'] = AbsTol
    sim_group['RelTol'] = RelTol
    dumbbell_group = sim_group.create_group("dumbbell")
    dumbbell_group["m1"] = 500
    dumbbell_group["m2"] = 500
    dumbbell_group['l'] = 0.003
    
    true_ast_group = sim_group.create_group("true_asteroid")
    true_ast_group.create_dataset("vertices", data=v, compression=compression,
                                  compression_opts=compression_opts)
    true_ast_group.create_dataset("faces", data=f, compression=compression,
                                  compression_opts=compression_opts)
    true_ast_group['name'] = 'castalia.obj'
    
    est_ast_group = sim_group.create_group("estimate_asteroid")
    est_ast_group['surf_area'] = surf_area
    est_ast_group['max_angle'] = max_angle
    est_ast_group['min_angle'] = min_angle
    est_ast_group['max_distance'] = max_distance
    est_ast_group['max_radius'] = max_radius
    est_ast_group.create_dataset('initial_vertices', data=ellipsoid.get_verts(), compression=compression,
                                 compression_opts=compression_opts)
    est_ast_group.create_dataset("initial_faces", data=ellipsoid.get_faces(), compression=compression,
                                 compression_opts=compression_opts)

    lidar_group = sim_group.create_group("lidar")
    lidar_group.create_dataset("view_axis", data=lidar.get_view_axis())
    lidar_group.create_dataset("up_axis", data=lidar.get_up_axis())
    lidar_group.create_dataset("fov", data=lidar.get_fov())
    
    return (true_ast_meshdata, true_ast, complete_controller, est_ast_meshdata, 
            est_ast_rmesh, est_ast, lidar, caster, max_angle, 
            dum, AbsTol, RelTol)

def simulate(output_filename="/tmp/exploration_sim.hdf5"):
    """Actually run the simulation around the asteroid
    """
    logger = logging.getLogger(__name__)

    num_steps = int(10)
    time = np.arange(0, num_steps)
    t0, tf = time[0], time[-1]
    dt = time[1] - time[0]

    # define the initial condition
    initial_pos = np.array([1.5, 0, 0])
    initial_vel = np.array([0, 0, 0])
    initial_R = attitude.rot3(np.pi / 2).reshape(-1)
    initial_w = np.array([0, 0, 0])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    with h5py.File(output_filename, 'w') as hf:
        hf.create_dataset('time', data=time, compression=compression,
                          compression_opts=compression_opts)
        hf.create_dataset("initial_state", data=initial_state, compression=compression,
                          compression_opts=compression_opts)

        v_group = hf.create_group("reconstructed_vertex")
        f_group = hf.create_group("reconstructed_face")
        w_group = hf.create_group("reconstructed_weight")
        state_group = hf.create_group("state")
        targets_group = hf.create_group("targets")
        Ra_group = hf.create_group("Ra")
        inertial_intersections_group = hf.create_group("inertial_intersections")
        asteroid_intersections_group = hf.create_group("asteroid_intersections")

        # initialize the simulation objects
        (true_ast_meshdata, true_ast, complete_controller,
         est_ast_meshdata, est_ast_rmesh, est_ast, lidar, caster, max_angle, dum,
         AbsTol, RelTol) = initialize(hf)
        
        # initialize the ODE function
        system = integrate.ode(eoms.eoms_controlled_inertial_pybind)
        system.set_integrator("lsoda", atol=AbsTol, rtol=RelTol)
        system.set_initial_value(initial_state, t0)
        system.set_f_params(true_ast, dum, complete_controller, est_ast_rmesh)
        
        point_cloud = defaultdict(list)

        ii = 1
        while system.successful() and system.t < tf:
            # integrate the system
            t = (system.t + dt)
            state = system.integrate(system.t + dt)

            logger.info("Step: {} Time: {}".format(ii, t))
            
            if not (np.floor(t) % 1):
                # logger.info("RayCasting at t: {}".format(t))

                targets = lidar.define_targets(state[0:3],
                                               state[6:15].reshape((3, 3)),
                                               np.linalg.norm(state[0:3]))
                # update the asteroid inside the caster
                nv = true_ast.rotate_vertices(t)
                Ra = true_ast.rot_ast2int(t)
                
                true_ast_meshdata.update_mesh(nv, true_ast_meshdata.get_faces())
                caster.update_mesh(true_ast_meshdata)

                # do the raycasting
                intersections = caster.castarray(state[0:3], targets)

                # reconstruct the mesh with new measurements
                # convert the intersections to the asteroid frame
                ast_ints = []
                for pt in intersections:
                    if np.linalg.norm(pt) < 1e-9:
                        logger.info("No intersection for this point")
                        pt_ast = np.array([np.nan, np.nan, np.nan])
                    else:
                        pt_ast = Ra.T.dot(pt)

                    ast_ints.append(pt_ast)
                
                ast_ints = np.array(ast_ints)

                est_ast_rmesh.update(ast_ints, max_angle)

                # save data to HDF5

            v_group.create_dataset(str(ii), data=est_ast_rmesh.get_verts(), compression=compression,
                                   compression_opts=compression_opts)
            f_group.create_dataset(str(ii), data=est_ast_rmesh.get_faces(), compression=compression,
                                   compression_opts=compression_opts)
            w_group.create_dataset(str(ii), data=est_ast_rmesh.get_weights(), compression=compression,
                                   compression_opts=compression_opts)

            state_group.create_dataset(str(ii), data=state, compression=compression,
                                       compression_opts=compression_opts)
            targets_group.create_dataset(str(ii), data=targets, compression=compression,
                                         compression_opts=compression_opts)
            Ra_group.create_dataset(str(ii), data=Ra, compression=compression,
                                    compression_opts=compression_opts)
            inertial_intersections_group.create_dataset(str(ii), data=intersections, compression=compression,
                                                        compression_opts=compression_opts)
            asteroid_intersections_group.create_dataset(str(ii), data=ast_ints, compression=compression,
                                                        compression_opts=compression_opts)
            
            ii += 1

def simulate_control(output_filename="/tmp/exploration_sim.hdf5"):
    """Run the simulation with the control cost added in
    """
    logger = logging.getLogger(__name__)
    
    num_steps = int(10)
    time = np.arange(0, num_steps)
    t0, tf = time[0], time[-1]
    dt = time[1] - time[0]

    # define the initial condition
    initial_pos = np.array([1.5, 0, 0])
    initial_vel = np.array([0, 0, 0])
    initial_R = attitude.rot3(np.pi / 2).reshape(-1)
    initial_w = np.array([0, 0, 0])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    with h5py.File(output_filename, 'w') as hf:
        hf.create_dataset('time', data=time, compression=compression,
                          compression_opts=compression_opts)
        hf.create_dataset("initial_state", data=initial_state, compression=compression,
                          compression_opts=compression_opts)

        v_group = hf.create_group("reconstructed_vertex")
        f_group = hf.create_group("reconstructed_face")
        w_group = hf.create_group("reconstructed_weight")
        state_group = hf.create_group("state")
        targets_group = hf.create_group("targets")
        Ra_group = hf.create_group("Ra")
        inertial_intersections_group = hf.create_group("inertial_intersections")
        asteroid_intersections_group = hf.create_group("asteroid_intersections")

        # initialize the simulation objects
        (true_ast_meshdata, true_ast, complete_controller,
         est_ast_meshdata, est_ast_rmesh, est_ast, lidar, caster, max_angle, dum,
         AbsTol, RelTol) = initialize(hf)

        # initialize the ODE function
        system = integrate.ode(eoms.eoms_controlled_inertial_control_cost_pybind)
        system.set_integrator("lsoda", atol=AbsTol, rtol=RelTol)
        system.set_initial_value(initial_state, t0)
        system.set_f_params(true_ast, dum, complete_controller, est_ast_rmesh, est_ast)
        
        point_cloud = defaultdict(list)

        ii = 1
        while system.successful() and system.t < tf:
            t = system.t + dt
            state = system.integrate(system.t + dt)

            logger.info("Step: {} Time: {}".format(ii, t))





def animate(filename):
    """Given a HDF5 file from simulate this will animate teh motion
    """
    # TODO Animate the changing of the mesh itself as a function of time
    with h5py.File(filename, 'r') as hf:
        # get the inertial state and asteroid mesh object
        time = hf['time'][()]
        state_group = hf['state']
        state_keys = np.array(utilities.sorted_nicely(list(hf['state'].keys())))
        
        intersections_group = hf['inertial_intersections']

        # extract out the entire state and intersections
        state = []
        inertial_intersections = []
        for key in state_keys:
            state.append(state_group[key][()])
            inertial_intersections.append(intersections_group[key][()])
        
        state = np.array(state)
        # get the true asteroid from the HDF5 file
        true_vertices = hf['simulation_parameters/true_asteroid/vertices'][()]
        true_faces = hf['simulation_parameters/true_asteroid/faces'][()]
        true_name = hf['simulation_parameters/true_asteroid/name'][()]
        
        est_initial_vertices = hf['simulation_parameters/estimate_asteroid/initial_vertices'][()]
        est_initial_faces = hf['simulation_parameters/estimate_asteroid/initial_faces'][()]

        mfig = graphics.mayavi_figure(size=(800,600))
        mesh, ast_axes = graphics.draw_polyhedron_mayavi(true_vertices, true_faces, mfig)

        # initialize a dumbbell object
        dum = dumbbell.Dumbbell(hf['simulation_parameters/dumbbell/m1'][()], 
                                hf['simulation_parameters/dumbbell/m2'][()],
                                hf['simulation_parameters/dumbbell/l'][()])
        com, dum_axes = graphics.draw_dumbbell_mayavi(state[0, :], dum, mfig)

        pc_lines = [graphics.mayavi_addLine(mfig, state[0, 0:3], p)
                    for p in inertial_intersections[0]]
        
        ast_meshdata = mesh_data.MeshData(true_vertices, true_faces)
        ast_meshparam = asteroid.MeshParam(ast_meshdata)
        ast = asteroid.Asteroid('castalia', ast_meshparam)
        
        # reconstructed vertices/faces groups
        rv_group = hf['reconstructed_vertex']
        rf_group = hf['reconstructed_face']
        
        mayavi_objects = (mesh, ast_axes, com, dum_axes, pc_lines)

    animation.inertial_asteroid_trajectory_cpp(time, state, inertial_intersections,
                                               filename, mayavi_objects)

def reconstruct_images(filename):
    """Read teh HDF5 data and generate a bunch of images of the reconstructing 
    asteroid
    """
    logger = logging.getLogger(__name__)

# TODO Add plotting functions from raycasting_sim.py
if __name__ == "__main__":
    # logging_file = tempfile.mkstemp(suffix='.txt.')[1]
    logging_file = "/tmp/exploration_log.txt"

    logging.basicConfig(filename=logging_file,
                        filemode='w', level=logging.INFO,
                        format='%(asctime)s %(levelname)-8s %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    print("Logging to {}".format(logging_file))

    parser = argparse.ArgumentParser(description="Exploration and asteroid reconstruction simulation",
                                      formatter_class=argparse.RawTextHelpFormatter)
    
    parser.add_argument("simulation_data",
                        help="Filename to store the simulation data")
    # parser.add_argument("reconstruct_data",
    #                     help="Filename to store the reconstruction data")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("-s", "--simulate", help="Run the exploration simulation",
                       action="store_true")
    group.add_argument("-a", "--animate", help="Animate the data from the exploration sim",
                       action="store_true")
    group.add_argument("-r", "--reconstruct", help="Generate images for the reconstruction",
                       action="store_true")

    args = parser.parse_args()
                                                                
    if args.simulate:
        simulate(args.simulation_data)
    elif args.reconstruct:
        output_path = tempfile.mkdtemp()
    elif args.animate:
        animate(args.simulation_data)


