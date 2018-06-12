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
import itertools
import subprocess

import h5py
import numpy as np
from scipy import integrate, interpolate
import matplotlib.pyplot as plt

from lib import asteroid, surface_mesh, cgal, mesh_data, reconstruct
from lib import surface_mesh
from lib import controller as controller_cpp
from lib import stats

from dynamics import dumbbell, eoms, controller
from point_cloud import wavefront
from kinematics import attitude
import utilities
from visualization import graphics, animation, publication

compression = 'gzip'
compression_opts = 9
max_steps = 10

def initialize(output_filename):
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
    true_ast = asteroid.Asteroid('castalia', true_ast_meshdata)

    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
    
    # estimated asteroid (starting as an ellipse)
    surf_area = 0.01
    max_angle = np.sqrt(surf_area / true_ast.get_axes()[0]**2)
    min_angle = 10
    max_distance = 0.5
    max_radius = 0.03

    ellipsoid = surface_mesh.SurfMesh(true_ast.get_axes()[0], true_ast.get_axes()[1], true_ast.get_axes()[2],
                                      min_angle, max_radius, max_distance)
    
    v_est = ellipsoid.get_verts()
    f_est = ellipsoid.get_faces()
    est_ast_meshdata = mesh_data.MeshData(v_est, f_est)
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
    caster = cgal.RayCaster(v, f)
    
    # save a bunch of parameters to the HDF5 file
    with h5py.File(output_filename, 'w-') as hf:
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
        est_ast_group.create_dataset('initial_vertices', data=est_ast_rmesh.get_verts(), compression=compression,
                                    compression_opts=compression_opts)
        est_ast_group.create_dataset("initial_faces", data=est_ast_rmesh.get_faces(), compression=compression,
                                    compression_opts=compression_opts)
        est_ast_group.create_dataset("initial_weight", data=est_ast_rmesh.get_weights(), compression=compression,
                                    compression_opts=compression_opts)

        lidar_group = sim_group.create_group("lidar")
        lidar_group.create_dataset("view_axis", data=lidar.get_view_axis())
        lidar_group.create_dataset("up_axis", data=lidar.get_up_axis())
        lidar_group.create_dataset("fov", data=lidar.get_fov())
    
    return (true_ast_meshdata, true_ast, complete_controller, est_ast_meshdata, 
            est_ast_rmesh, est_ast, lidar, caster, max_angle, 
            dum, AbsTol, RelTol)

def initialize_eros(output_filename):
    """Initialize all the things for the simulation around Itokwawa

    Output_file : the actual HDF5 file to save the data/parameters to

    """
    logger = logging.getLogger(__name__)
    logger.info('Initialize asteroid and dumbbell objects')

    AbsTol = 1e-9
    RelTol = 1e-9
    
    # true asteroid and dumbbell
    v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')

    true_ast_meshdata = mesh_data.MeshData(v, f)
    true_ast = asteroid.Asteroid('itokawa', true_ast_meshdata)

    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
    
    # estimated asteroid (starting as an ellipse)
    surf_area = 0.01
    max_angle = np.sqrt(surf_area / true_ast.get_axes()[0]**2)
    min_angle = 10
    max_distance = 0.5
    max_radius = 0.03

    ellipsoid = surface_mesh.SurfMesh(true_ast.get_axes()[0], true_ast.get_axes()[1], true_ast.get_axes()[2],
                                      min_angle, max_radius, max_distance)
    
    v_est = ellipsoid.get_verts()
    f_est = ellipsoid.get_faces()
    est_ast_meshdata = mesh_data.MeshData(v_est, f_est)
    est_ast_rmesh = reconstruct.ReconstructMesh(est_ast_meshdata)
    est_ast = asteroid.Asteroid("itokawa", est_ast_rmesh)

    # controller functions 
    complete_controller = controller_cpp.Controller()
    
    # lidar object
    lidar = cgal.Lidar()
    lidar = lidar.view_axis(np.array([1, 0, 0]))
    lidar = lidar.up_axis(np.array([0, 0, 1]))
    lidar = lidar.fov(np.deg2rad(np.array([7, 7]))).dist(2).num_steps(3)

    # raycaster from c++
    caster = cgal.RayCaster(v, f)
    
    # save a bunch of parameters to the HDF5 file
    with h5py.File(output_filename, 'w-') as hf:
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
        est_ast_group.create_dataset('initial_vertices', data=est_ast_rmesh.get_verts(), compression=compression,
                                    compression_opts=compression_opts)
        est_ast_group.create_dataset("initial_faces", data=est_ast_rmesh.get_faces(), compression=compression,
                                    compression_opts=compression_opts)
        est_ast_group.create_dataset("initial_weight", data=est_ast_rmesh.get_weights(), compression=compression,
                                    compression_opts=compression_opts)

        lidar_group = sim_group.create_group("lidar")
        lidar_group.create_dataset("view_axis", data=lidar.get_view_axis())
        lidar_group.create_dataset("up_axis", data=lidar.get_up_axis())
        lidar_group.create_dataset("fov", data=lidar.get_fov())
    
    return (true_ast_meshdata, true_ast, complete_controller, est_ast_meshdata, 
            est_ast_rmesh, est_ast, lidar, caster, max_angle, 
            dum, AbsTol, RelTol)

def initialize_itokawa(output_filename):
    """Initialize all the things for the simulation around Itokwawa

    Output_file : the actual HDF5 file to save the data/parameters to

    """
    logger = logging.getLogger(__name__)
    logger.info('Initialize asteroid and dumbbell objects')

    AbsTol = 1e-9
    RelTol = 1e-9
    
    # true asteroid and dumbbell
    v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')

    true_ast_meshdata = mesh_data.MeshData(v, f)
    true_ast = asteroid.Asteroid('itokawa', true_ast_meshdata)

    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
    
    # estimated asteroid (starting as an ellipse)
    surf_area = 0.01
    max_angle = np.sqrt(surf_area / true_ast.get_axes()[0]**2)
    min_angle = 10
    max_distance = 0.1
    max_radius = 0.006

    ellipsoid = surface_mesh.SurfMesh(true_ast.get_axes()[0], true_ast.get_axes()[1], true_ast.get_axes()[2],
                                      min_angle, max_radius, max_distance)
    
    v_est = ellipsoid.get_verts()
    f_est = ellipsoid.get_faces()
    est_ast_meshdata = mesh_data.MeshData(v_est, f_est)
    est_ast_rmesh = reconstruct.ReconstructMesh(est_ast_meshdata)
    est_ast = asteroid.Asteroid("itokawa", est_ast_rmesh)

    # controller functions 
    complete_controller = controller_cpp.Controller()
    
    # lidar object
    lidar = cgal.Lidar()
    lidar = lidar.view_axis(np.array([1, 0, 0]))
    lidar = lidar.up_axis(np.array([0, 0, 1]))
    lidar = lidar.fov(np.deg2rad(np.array([7, 7]))).dist(2).num_steps(3)

    # raycaster from c++
    caster = cgal.RayCaster(v, f)
    
    # save a bunch of parameters to the HDF5 file
    with h5py.File(output_filename, 'w-') as hf:
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
        est_ast_group.create_dataset('initial_vertices', data=est_ast_rmesh.get_verts(), compression=compression,
                                    compression_opts=compression_opts)
        est_ast_group.create_dataset("initial_faces", data=est_ast_rmesh.get_faces(), compression=compression,
                                    compression_opts=compression_opts)
        est_ast_group.create_dataset("initial_weight", data=est_ast_rmesh.get_weights(), compression=compression,
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

    num_steps = int(max_steps)
    time = np.arange(0, num_steps)
    t0, tf = time[0], time[-1]
    dt = time[1] - time[0]

    # define the initial condition in the inertial frame
    initial_pos = np.array([1.5, 0, 0])
    initial_vel = np.array([0, 0, 0])
    initial_R = attitude.rot3(np.pi / 2).reshape(-1)
    initial_w = np.array([0, 0, 0])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # initialize the simulation objects
    (true_ast_meshdata, true_ast, complete_controller,
        est_ast_meshdata, est_ast_rmesh, est_ast, lidar, caster, max_angle, dum,
        AbsTol, RelTol) = initialize(output_filename)
    
    with h5py.File(output_filename, 'a') as hf:
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

        
        # initialize the ODE function
        system = integrate.ode(eoms.eoms_controlled_inertial_pybind)
        system.set_integrator("lsoda", atol=AbsTol, rtol=RelTol, nsteps=10000)
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
                
                # this also updates true_ast (both point to same data) 
                caster.update_mesh(nv, true_ast.get_faces())

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
                
                # convert the intersections to the asteroid frame
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
    
    num_steps = int(max_steps)
    time = np.arange(0, num_steps)
    t0, tf = time[0], time[-1]
    dt = time[1] - time[0]

    # define the initial condition in the inertial frame
    # initial_pos = np.array([1.5, 0, 0]) # castalia
    initial_pos = np.array([1.5, 0, 0])
    initial_vel = np.array([0, 0, 0])
    initial_R = attitude.rot3(np.pi / 2).reshape(-1)
    initial_w = np.array([0, 0, 0])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # initialize the simulation objects
    (true_ast_meshdata, true_ast, complete_controller,
        est_ast_meshdata, est_ast_rmesh, est_ast, lidar, caster, max_angle, dum,
        AbsTol, RelTol) = initialize_itokawa(output_filename)

    with h5py.File(output_filename, 'a') as hf:
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


        # initialize the ODE function
        system = integrate.ode(eoms.eoms_controlled_inertial_control_cost_pybind)
        system.set_integrator("lsoda", atol=AbsTol, rtol=RelTol, nsteps=10000)
        # system.set_integrator("vode", nsteps=5000, method='bdf')
        system.set_initial_value(initial_state, t0)
        system.set_f_params(true_ast, dum, complete_controller, est_ast_rmesh, est_ast)
        
        point_cloud = defaultdict(list)

        ii = 1
        while system.successful() and system.t < tf:
            t = system.t + dt
            # TODO Make sure the asteroid (est and truth) are being rotated by ROT3(t)
            state = system.integrate(system.t + dt)

            logger.info("Step: {} Time: {} Uncertainty: {}".format(ii, t, 
                                                                   np.sum(est_ast_rmesh.get_weights())))

            if not (np.floor(t) % 1):
                targets = lidar.define_targets(state[0:3],
                                               state[6:15].reshape((3, 3)),
                                               np.linalg.norm(state[0:3]))

                # update the asteroid inside the caster
                nv = true_ast.rotate_vertices(t)
                Ra = true_ast.rot_ast2int(t)
                
                caster.update_mesh(nv, true_ast.get_faces())

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
                
                # this updates the estimated asteroid mesh used in both rmesh and est_ast
                est_ast_rmesh.update(ast_ints, max_angle)

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
        
        logger.info("Exploration complete")
        logger.info("Estimated asteroid has {} vertices and {} faces".format(
            est_ast_rmesh.get_verts().shape[0],
            est_ast_rmesh.get_faces().shape[1]))
            
        logger.info("Now refining the faces close to the landing site")
        # perform remeshing over the landing area and take a bunch of measurements 
        # of the surface. Assume everything happens without the asteroid rotating 
        new_face_centers = est_ast_meshdata.refine_faces_in_view(state[0:3], np.deg2rad(15))
        logger.info("Refinement added {} faces".format(new_face_centers.shape[0]))
        logger.info("Estimated asteroid has {} vertices and {} faces".format(
            est_ast_rmesh.get_verts().shape[0],
            est_ast_rmesh.get_faces().shape[1]))
        
        logger.info("Now looping over the new faces and raycasting")
        # now take measurements of each facecenter
        for ii, vec in enumerate(new_face_centers):
            logger.info("Step: {} Uncertainty: {}".format(ii + max_steps, 
                                                                   np.sum(est_ast_rmesh.get_weights())))

            targets = lidar.define_targets(state[0:3], state[6:15].reshape((3, 3)),
                                           np.linalg.norm(state[0:3]))
            intersections = caster.castarray(state[0:3], targets)
            ast_ints = []
            for pt in intersections:
                if np.linalg.norm(pt) < 1e-9:
                    logger.info("No intersection for this point")
                    pt_ast = np.array([np.nan, np.nan, np.nan])
                else:
                    pt_ast = Ra.T.dot(pt)

                ast_ints.append(pt_ast)
            
            ast_ints = np.array(ast_ints)

            # this updates the estimated asteroid mesh used in both rmesh and est_ast
            est_ast_rmesh.update(ast_ints, max_angle)
            
            # use the controller to update the state
            complete_controller.inertial_fixed_state(max_steps, state, initial_pos);
            complete_controller.inertial_pointing_attitude(max_steps,
                                                           state,
                                                           vec);
            # update the state
            state = np.hstack((complete_controller.get_posd(), 
                               complete_controller.get_veld(),
                               complete_controller.get_Rd().reshape(-1),
                               complete_controller.get_ang_vel_d()))

            v_group.create_dataset(str(max_steps + ii), data=est_ast_rmesh.get_verts(), compression=compression,
                                   compression_opts=compression_opts)
            f_group.create_dataset(str(max_steps + ii), data=est_ast_rmesh.get_faces(), compression=compression,
                                   compression_opts=compression_opts)
            w_group.create_dataset(str(max_steps + ii), data=est_ast_rmesh.get_weights(), compression=compression,
                                   compression_opts=compression_opts)

            state_group.create_dataset(str(max_steps + ii), data=state, compression=compression,
                                       compression_opts=compression_opts)
            targets_group.create_dataset(str(max_steps + ii), data=targets, compression=compression,
                                         compression_opts=compression_opts)
            Ra_group.create_dataset(str(max_steps + ii), data=Ra, compression=compression,
                                    compression_opts=compression_opts)
            inertial_intersections_group.create_dataset(str(max_steps + ii), data=intersections, compression=compression,
                                                        compression_opts=compression_opts)
            asteroid_intersections_group.create_dataset(str(max_steps + ii), data=ast_ints, compression=compression,
                                                        compression_opts=compression_opts)

    
    logger.info("All done")

def save_animation(filename, move_cam=False, mesh_weight=False):
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
        inertial_intersections = np.array(inertial_intersections)
        # get the true asteroid from the HDF5 file
        true_vertices = hf['simulation_parameters/true_asteroid/vertices'][()]
        true_faces = hf['simulation_parameters/true_asteroid/faces'][()]
        true_name = hf['simulation_parameters/true_asteroid/name'][()]
        
        est_initial_vertices = hf['simulation_parameters/estimate_asteroid/initial_vertices'][()]
        est_initial_faces = hf['simulation_parameters/estimate_asteroid/initial_faces'][()]
            
        # think about a black background as well
        mfig = graphics.mayavi_figure(size=(800,600), offscreen=True)
        
        if mesh_weight:
            mesh = graphics.mayavi_addMesh(mfig, est_initial_vertices, est_initial_faces,
                                           scalars=np.squeeze(hf['simulation_parameters/estimate_asteroid/initial_weight'][()]),
                                           color=None, colormap='viridis')
        else:
            mesh = graphics.mayavi_addMesh(mfig, est_initial_vertices, est_initial_faces)

        xaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([2, 0, 0]), color=(1, 0, 0)) 
        yaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([0, 2, 0]), color=(0, 1, 0)) 
        zaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([0, 0, 2]), color=(0, 0, 1)) 
        ast_axes = (xaxis, yaxis, zaxis)
        # initialize a dumbbell object
        dum = dumbbell.Dumbbell(hf['simulation_parameters/dumbbell/m1'][()], 
                                hf['simulation_parameters/dumbbell/m2'][()],
                                hf['simulation_parameters/dumbbell/l'][()])
        # com, dum_axes = graphics.draw_dumbbell_mayavi(state[0, :], dum, mfig)
        if move_cam:
            com = graphics.mayavi_addPoint(mfig, state[0, 0:3],
                                           color=(1, 0, 0), radius=0.02,
                                           opacity=0.5)
        else:
            com = graphics.mayavi_addPoint(mfig, state[0, 0:3],
                                           color=(1, 0, 0), radius=0.1)

        pc_points = graphics.mayavi_points3d(mfig, inertial_intersections[0], 
                                             color=(0, 0, 1), scale_factor=0.05)
        
        # add some text objects
        time_text = graphics.mlab.text(0.1, 0.1, "t: {:8.1f}".format(0), figure=mfig,
                                       color=(0, 0, 0), width=0.05)
        weight_text = graphics.mlab.text(0.1, 0.2, "w: {:8.1f}".format(0), figure=mfig,
                                         color=(0, 0, 0), width=0.05)
        # mayavi_objects = (mesh, ast_axes, com, dum_axes, pc_lines)
        mayavi_objects = (mesh, com, pc_points, time_text, weight_text)
    
        output_path = tempfile.mkdtemp()
        print("Images will be saved to {}".format(output_path))

    animation.inertial_asteroid_trajectory_cpp_save(time, state, inertial_intersections,
                                                    filename, mayavi_objects, move_cam=move_cam,
                                                    mesh_weight=mesh_weight,
                                                    output_path=output_path)
    # now call ffmpeg
    fps = 60
    name = 'exploration'
    ffmpeg_fname = os.path.join(output_path, '%07d.jpg')
    cmd = "ffmpeg -framerate {} -i {} -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p -vf 'scale=trunc(iw/2)*2:trunc(ih/2)*2' {}.mp4".format(fps, ffmpeg_fname, name)
    print(cmd)
    subprocess.check_output(['bash', '-c', cmd])

    # remove folder now
    for file in os.listdir(output_path): 
        file_path = os.path.join(output_path, file)
        if os.path.isfile(file_path):
            os.remove(file_path)
    os.rmdir(output_path)

def animate(filename, move_cam=False, mesh_weight=False, save_animation=False):
    """Given a HDF5 file from simulate this will animate teh motion
    """
    # TODO Animate the changing of the mesh itself as a function of time
    with h5py.File(filename, 'r') as hf:
        # get the inertial state and asteroid mesh object
        # time = hf['time'][()]
        state_group = hf['state']
        state_keys = np.array(utilities.sorted_nicely(list(hf['state'].keys())))
        time = [int(t) for t in state_keys];

        intersections_group = hf['inertial_intersections']

        # extract out the entire state and intersections
        state = []
        inertial_intersections = []
        for key in state_keys:
            state.append(state_group[key][()])
            inertial_intersections.append(intersections_group[key][()])
        
        state = np.array(state)
        inertial_intersections = np.array(inertial_intersections)
        # get the true asteroid from the HDF5 file
        true_vertices = hf['simulation_parameters/true_asteroid/vertices'][()]
        true_faces = hf['simulation_parameters/true_asteroid/faces'][()]
        true_name = hf['simulation_parameters/true_asteroid/name'][()]
        
        est_initial_vertices = hf['simulation_parameters/estimate_asteroid/initial_vertices'][()]
        est_initial_faces = hf['simulation_parameters/estimate_asteroid/initial_faces'][()]
            
        # think about a black background as well
        mfig = graphics.mayavi_figure(size=(800,600))
        
        if mesh_weight:
            mesh = graphics.mayavi_addMesh(mfig, est_initial_vertices, est_initial_faces,
                                           scalars=np.squeeze(hf['simulation_parameters/estimate_asteroid/initial_weight'][()]),
                                           color=None, colormap='viridis')
        else:
            mesh = graphics.mayavi_addMesh(mfig, est_initial_vertices, est_initial_faces)

        xaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([2, 0, 0]), color=(1, 0, 0)) 
        yaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([0, 2, 0]), color=(0, 1, 0)) 
        zaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([0, 0, 2]), color=(0, 0, 1)) 
        ast_axes = (xaxis, yaxis, zaxis)
        # initialize a dumbbell object
        dum = dumbbell.Dumbbell(hf['simulation_parameters/dumbbell/m1'][()], 
                                hf['simulation_parameters/dumbbell/m2'][()],
                                hf['simulation_parameters/dumbbell/l'][()])
        # com, dum_axes = graphics.draw_dumbbell_mayavi(state[0, :], dum, mfig)
        if move_cam:
            com = graphics.mayavi_addPoint(mfig, state[0, 0:3],
                                           color=(1, 0, 0), radius=0.02,
                                           opacity=0.5)
        else:
            com = graphics.mayavi_addPoint(mfig, state[0, 0:3],
                                           color=(1, 0, 0), radius=0.1)

        pc_points = graphics.mayavi_points3d(mfig, inertial_intersections[0], 
                                             color=(0, 0, 1), scale_factor=0.05)
        
        # add some text objects
        time_text = graphics.mlab.text(0.1, 0.1, "t: {:8.1f}".format(0), figure=mfig,
                                       color=(0, 0, 0), width=0.05)
        weight_text = graphics.mlab.text(0.1, 0.2, "w: {:8.1f}".format(0), figure=mfig,
                                         color=(0, 0, 0), width=0.05)
        # mayavi_objects = (mesh, ast_axes, com, dum_axes, pc_lines)
        mayavi_objects = (mesh, com, pc_points, time_text, weight_text)
    
    animation.inertial_asteroid_trajectory_cpp(time, state, inertial_intersections,
                                               filename, mayavi_objects, move_cam=move_cam,
                                               mesh_weight=mesh_weight)
    graphics.mlab.show()

def animate_landing(filename, move_cam=False, mesh_weight=False):
    """Animation for the landing portion of simulation
    """
    with h5py.File(filename, 'r') as hf:
        time = hf['time'][()]
        state_group = hf['state']
        state_keys = np.array(utilities.sorted_nicely(list(state_group.keys())))

        state = []
        for key in state_keys:
            state.append(state_group[key][()])

        state=np.array(state)

        mfig = graphics.mayavi_figure(size=(800, 600))

        # option for the mesh weight
        if mesh_weight:
            mesh = graphics.mayavi_addMesh(mfig, hf['vertices'][()], hf['faces'][()],
                                           scalars=np.squeeze(hf['weight'][()]),
                                           color=None, colormap='viridis')
        else:
            mesh = graphics.mayavi_addMesh(mfig, hf['vertices'][()], hf['faces'][()])

        xaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([2, 0, 0]), color=(1, 0, 0)) 
        yaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([0, 2, 0]), color=(0, 1, 0)) 
        zaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([0, 0, 2]), color=(0, 0, 1)) 
        ast_axes = (xaxis, yaxis, zaxis)

        if move_cam:
            com = graphics.mayavi_addPoint(mfig, state[0, 0:3],
                                           color=(1, 0, 0), radius=0.02,
                                           opacity=0.5)
        else:
            com = graphics.mayavi_addPoint(mfig, state[0, 0:3],
                                           color=(1, 0, 0), radius=0.1)

        # add some text objects
        time_text = graphics.mlab.text(0.1, 0.1, "t: {:8.1f}".format(0), figure=mfig,
                                       color=(0, 0, 0), width=0.05)
        weight_text = graphics.mlab.text(0.1, 0.2, "w: {:8.1f}".format(0), figure=mfig,
                                         color=(0, 0, 0), width=0.05)

        mayavi_objects = (mesh, com, time_text, weight_text)

    animation.inertial_asteroid_landing_cpp(time, state, filename, mayavi_objects, 
                                            move_cam=move_cam, mesh_weight=mesh_weight)
    graphics.mlab.show()

def save_animate_landing(filename, move_cam=False, mesh_weight=False):
    """Save the landing animation
    """

    with h5py.File(filename, 'r') as hf:
        time = hf['time'][()]
        state_group = hf['state']
        state_keys = np.array(utilities.sorted_nicely(list(state_group.keys())))

        state = []
        for key in state_keys:
            state.append(state_group[key][()])

        state=np.array(state)

        mfig = graphics.mayavi_figure(size=(800, 600), offscreen=True)

        # option for the mesh weight
        if mesh_weight:
            mesh = graphics.mayavi_addMesh(mfig, hf['vertices'][()], hf['faces'][()],
                                           scalars=np.squeeze(hf['weight'][()]),
                                           color=None, colormap='viridis')
        else:
            mesh = graphics.mayavi_addMesh(mfig, hf['vertices'][()], hf['faces'][()])

        xaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([2, 0, 0]), color=(1, 0, 0)) 
        yaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([0, 2, 0]), color=(0, 1, 0)) 
        zaxis = graphics.mayavi_addLine(mfig, np.array([0, 0, 0]), np.array([0, 0, 2]), color=(0, 0, 1)) 
        ast_axes = (xaxis, yaxis, zaxis)

        if move_cam:
            com = graphics.mayavi_addPoint(mfig, state[0, 0:3],
                                           color=(1, 0, 0), radius=0.02,
                                           opacity=0.5)
        else:
            com = graphics.mayavi_addPoint(mfig, state[0, 0:3],
                                           color=(1, 0, 0), radius=0.1)

        # add some text objects
        time_text = graphics.mlab.text(0.1, 0.1, "t: {:8.1f}".format(0), figure=mfig,
                                       color=(0, 0, 0), width=0.05)
        weight_text = graphics.mlab.text(0.1, 0.2, "w: {:8.1f}".format(0), figure=mfig,
                                         color=(0, 0, 0), width=0.05)

        mayavi_objects = (mesh, com, time_text, weight_text)

        output_path = tempfile.mkdtemp()
        print("Images will be saved to {}".format(output_path))

    animation.inertial_asteroid_landing_cpp_save(time, state, filename, mayavi_objects, 
                                                 move_cam=move_cam, mesh_weight=mesh_weight,
                                                 output_path=output_path)
    # now call ffmpeg
    fps = 60
    name = 'landing'
    ffmpeg_fname = os.path.join(output_path, '%07d.jpg')
    cmd = "ffmpeg -framerate {} -i {} -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p -vf 'scale=trunc(iw/2)*2:trunc(ih/2)*2' {}.mp4".format(fps, ffmpeg_fname, name)
    print(cmd)
    subprocess.check_output(['bash', '-c', cmd])

    # remove folder now
    for file in os.listdir(output_path): 
        file_path = os.path.join(output_path, file)
        if os.path.isfile(file_path):
            os.remove(file_path)
    os.rmdir(output_path)


def landing(output_filename, input_filename):
    """Open the HDF5 file and continue the simulation from the terminal state
    to landing on the surface over an additional few hours
    """
    logger = logging.getLogger(__name__)

    logger.info("Opening the HDF5 file from exploration {}".format(input_filename))
    
    # TODO Look at blender_sim
    # get all the terminal states from the exploration stage
    with h5py.File(input_filename, 'r') as hf:
        state_keys = np.array(utilities.sorted_nicely(list(hf['state'].keys())))
        # explore_tf = hf['time'][()][-1]
        explore_tf = int(state_keys[-1])
        explore_state = hf['state/' + str(explore_tf)][()]
        explore_Ra = hf['Ra/' + str(explore_tf)][()]
        explore_v = hf['reconstructed_vertex/' + str(explore_tf)][()]
        explore_f = hf['reconstructed_face/' + str(explore_tf)][()]
        explore_w = hf['reconstructed_weight/' + str(explore_tf)][()]
        
        explore_name = hf['simulation_parameters/true_asteroid/name'][()][:-4]
        explore_m1 = hf['simulation_parameters/dumbbell/m1'][()]
        explore_m2 = hf['simulation_parameters/dumbbell/m2'][()]
        explore_l = hf['simulation_parameters/dumbbell/l'][()]
        explore_AbsTol = hf['simulation_parameters/AbsTol'][()]
        explore_RelTol = hf['simulation_parameters/RelTol'][()]
        
        explore_true_vertices = hf['simulation_parameters/true_asteroid/vertices'][()]
        explore_true_faces = hf['simulation_parameters/true_asteroid/faces'][()]
    
    num_steps = int(3600) # 2 hours to go from home pos to the surface
    time = np.arange(max_steps,max_steps  + num_steps)
    t0, tf = time[0], time[-1]
    dt = time[1] - time[0]
    
    # initialize the asteroid and dumbbell objects
    true_ast_meshdata = mesh_data.MeshData(explore_true_vertices, explore_true_faces)
    true_ast = asteroid.Asteroid('castalia', true_ast_meshdata)
    
    est_ast_meshdata = mesh_data.MeshData(explore_v, explore_f)
    est_ast = asteroid.Asteroid('castalia', est_ast_meshdata)

    # find the face with the lowest slope in our landing region
    initial_ast_pos = explore_Ra.dot(explore_state[0:3])
    desired_asteroid_pos = est_ast.land_in_view(initial_ast_pos, np.deg2rad(15))

    dum = dumbbell.Dumbbell(m1=explore_m1, m2=explore_m2, l=explore_l)

    initial_state = explore_state
    with h5py.File(output_filename, 'w-') as hf:
        # save data to HDF5 file
        hf.create_dataset('time', data=time, compression=compression,
                          compression_opts=compression_opts)
        hf.create_dataset("initial_state", data=initial_state, compression=compression,
                          compression_opts=compression_opts)
        hf.create_dataset("vertices", data=explore_v, compression=compression,
                          compression_opts=compression_opts)
        hf.create_dataset("faces", data=explore_f, compression=compression,
                          compression_opts=compression_opts)
        hf.create_dataset("weight", data=explore_w, compression=compression,
                          compression_opts=compression_opts)

        state_group = hf.create_group("state")
        Ra_group = hf.create_group("Ra")

        # define the system EOMS and simulate
        system = integrate.ode(eoms.eoms_controlled_land_pybind)
        system.set_integrator("lsoda", atol=explore_AbsTol, rtol=explore_RelTol,  nsteps=10000)
        system.set_initial_value(initial_state, t0)
        system.set_f_params(true_ast, dum, est_ast, desired_asteroid_pos)
        
        ii = 1
        while system.successful() and system.t < tf:
            t = system.t + dt
            state = system.integrate(system.t + dt)

            logger.info("Step: {} Time: {}".format(ii, t))
            
            state_group.create_dataset(str(ii), data=state, compression=compression,
                                       compression_opts=compression_opts)
            Ra_group.create_dataset(str(ii), data=est_ast.rot_ast2int(t), compression=compression,
                                    compression_opts=compression_opts)
            ii+=1

def reconstruct_images(filename, output_path="/tmp/reconstruct_images"):
    """Read teh HDF5 data and generate a bunch of images of the reconstructing 
    asteroid
    """
    logger = logging.getLogger(__name__)
    logger.info("Starting image generation")
    
    magnification = 1
    offscreen = True
    # check if location exists
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    
    logger.info('Opening {}'.format(filename))
    with h5py.File(filename, 'r') as hf:
        rv = hf['reconstructed_vertex']
        rf = hf['reconstructed_face']
        rw = hf['reconstructed_weight']

        # get all the keys for the groups
        v_keys = np.array(utilities.sorted_nicely(list(rv.keys())))
        f_keys = np.array(utilities.sorted_nicely(list(rf.keys())))
        w_keys = np.array(utilities.sorted_nicely(list(rw.keys())))
        
        v_initial = hf['simulation_parameters/estimate_asteroid/initial_vertices'][()]
        f_initial = hf['simulation_parameters/estimate_asteroid/initial_faces'][()]
        w_initial = np.squeeze(hf['simulation_parameters/estimate_asteroid/initial_weight'][()])

        """Partial images during the reconstruction"""
        logger.info('Starting on partial reconstruction images')

        mfig = graphics.mayavi_figure(offscreen=offscreen)
        mesh = graphics.mayavi_addMesh(mfig, v_initial, f_initial)
        ms = mesh.mlab_source
        graphics.mayavi_axes(mfig, [-1, 1, -1, 1, -1, 1], line_width=5, color=(1, 0, 0))
        graphics.mayavi_view(fig=mfig)

        partial_index = np.array([1, v_keys.shape[0]*1/4, v_keys.shape[0]*1/2,
                                  v_keys.shape[0]*3/4, v_keys.shape[0]*4/4-1],
                                 dtype=np.int)
        for img_index, vk in enumerate(partial_index):
            filename = os.path.join(output_path, 'partial_' + str(vk) + '.jpg')
            v = rv[str(vk)][()]
            # generate an image and save it 
            ms.set(x=v[:, 0], y=v[:, 1], z=v[:,2], triangles=f_initial)
            graphics.mlab.savefig(filename, magnification=magnification)
        
        """Partial images using a colormap for the data"""
        logger.info('Now using a colormap for the uncertainty')
        mfig = graphics.mayavi_figure(offscreen=offscreen)
        mesh = graphics.mayavi_addMesh(mfig, v_initial, f_initial,
                                       color=None, colormap='viridis',
                                       scalars=w_initial)
        ms = mesh.mlab_source
        graphics.mayavi_axes(mfig, [-1, 1, -1, 1, -1, 1], line_width=5, color=(1, 0, 0))
        graphics.mayavi_view(fig=mfig)

        partial_index = np.array([1, v_keys.shape[0]*1/4, v_keys.shape[0]*1/2,
                                  v_keys.shape[0]*3/4, v_keys.shape[0]*4/4-1],
                                 dtype=np.int)
        for img_index, vk in enumerate(partial_index):
            filename = os.path.join(output_path, 'partial_weights_' + str(vk) + '.jpg')
            v = rv[str(vk)][()]
            w = np.squeeze(rw[str(vk)][()])
            # generate an image and save it 
            ms.set(x=v[:, 0], y=v[:, 1], z=v[:,2], triangles=f_initial,
                     scalars=w)
            graphics.mlab.savefig(filename, magnification=magnification)

        """Generate the completed shape at a variety of different angles"""
        logger.info('Now generating some views of the final shape')
        # change the mesh to the finished mesh
        ms.reset(x=rv[v_keys[-1]][()][:, 0],y=rv[v_keys[-1]][()][:, 1],z=rv[v_keys[-1]][()][:, 2],
                 triangles=f_initial)
        elevation = np.array([30, -30])
        azimuth = np.array([0, 45, 135, 215, 315])
    
        for az, el in itertools.product(azimuth, elevation):
            filename = os.path.join(output_path,'final_az=' + str(az) + '_el=' + str(el) + '.jpg')
            graphics.mayavi_view(fig=mfig, azimuth=az, elevation=el)
            graphics.mlab.savefig(filename, magnification=magnification)

        # """Create a bunch of images for animation"""
        # logger.info('Now making images for a movie')
        # animation_path = os.path.join(output_path, 'animation')
        # if not os.path.exists(animation_path):
        #     os.makedirs(animation_path)
        
        # ms.set(x=v_initial[:, 0], y=v_initial[:, 1], z=v_initial[:, 2], triangles=f_initial)

        # for ii, vk in enumerate(v_keys):
        #     filename = os.path.join(animation_path, str(ii).zfill(7) + '.jpg')
        #     v = rv[vk][()]
        #     ms.reset(x=v[:, 0], y=v[:, 1], z=v[:, 2], triangles=f_initial)
        #     graphics.mayavi_savefig(mfig, filename, magnification=magnification)
    

    logger.info('Finished')

    return mfig

def plot_uncertainty(filename):
    """Compute the sum of uncertainty and plot as function of time"""
    logger = logging.getLogger(__name__)
    logger.info("Uncertainty plot as funciton of time")

    with h5py.File(filename, 'r') as hf:
        rv = hf['reconstructed_vertex']
        rf = hf['reconstructed_face']
        rw = hf['reconstructed_weight']
        
        # get all the keys for the groups
        v_keys = np.array(utilities.sorted_nicely(list(rv.keys())))
        f_keys = np.array(utilities.sorted_nicely(list(rf.keys())))
        w_keys = np.array(utilities.sorted_nicely(list(rw.keys())))
        
        v_initial = hf['simulation_parameters/estimate_asteroid/initial_vertices'][()]
        f_initial = hf['simulation_parameters/estimate_asteroid/initial_faces'][()]
        w_initial = np.squeeze(hf['simulation_parameters/estimate_asteroid/initial_weight'][()])
    
        t_array = []
        w_array = []

        for ii, wk in enumerate(w_keys):
            logger.info("Step {}".format(ii))
            t_array.append(ii)
            w_array.append(np.sum(rw[wk][()]))

        t_array = np.array(t_array)
        w_array = np.array(w_array)
        
    logger.info("Plotting")
    publication.plot_uncertainty(t_array, w_array)

def animate_uncertainty(filename):
    """Create a 2D projection of the uncertainty of the surface as a function of 
    time
    """
    with h5py.File(filename, 'r') as hf:
        rv = hf['reconstructed_vertex']
        rw = hf['reconstructed_weight']
        
        # get all the keys for the groups
        v_keys = np.array(utilities.sorted_nicely(list(rv.keys())))
        w_keys = np.array(utilities.sorted_nicely(list(rw.keys())))
        
        v_initial = hf['simulation_parameters/estimate_asteroid/initial_vertices'][()]
        w_initial = np.squeeze(hf['simulation_parameters/estimate_asteroid/initial_weight'][()])

        # convert vertices to spherical
        vs_initial = wavefront.cartesian2spherical(v_initial)

        fig, ax = plt.subplots(1, 1)
        ax.contourf(vs_initial[:, 2], vs_initial[:, 1], np.diag(w_initial))
        plt.show()

def plot_state_trajectory(filename):
    """Plot the state trajectory of the satellite around the asteroid

    """

    with h5py.File(filename, 'r') as hf:
        state_group = hf['state']
        Ra_group = hf['Ra']

        state_keys = np.array(utilities.sorted_nicely(list(hf['state'].keys())))

        t_array = np.zeros(len(state_keys))
        state_inertial_array = np.zeros((len(state_keys), 18))
        state_asteroid_array = np.zeros((len(state_keys), 18))
        
        for ii, sk in enumerate(state_keys):
            t_array[ii] = ii
            state_inertial_array[ii, :] = state_group[sk][()]
            Ra = Ra_group[sk][()]
            pos_ast = Ra.T.dot(state_inertial_array[ii, 0:3].T).T
            vel_ast = Ra.T.dot(state_inertial_array[ii, 3:6].T).T
            R_sc2ast = Ra.T.dot(state_inertial_array[ii, 6:15].reshape((3, 3)))
            w_ast = state_inertial_array[ii, 15:18]

            state_asteroid_array[ii, :] = np.hstack((pos_ast, vel_ast, R_sc2ast.reshape(-1), w_ast)) 

    publication.plot_state(t_array, state_inertial_array, state_asteroid_array)

def plot_volume(filename):
    """Compute the volume of the asteroid at each time step
    """
    with h5py.File(filename, 'r') as hf:
        rv_group = hf['reconstructed_vertex']
        rf_group = hf['reconstructed_face']

        rv_keys = np.array(utilities.sorted_nicely(list(rv_group.keys())))

        t_array = np.zeros(len(rv_keys))
        vol_array = np.zeros(len(rv_keys))

        for ii, key in enumerate(rv_keys):
            t_array[ii] = ii
            vol_array[ii] = stats.volume(rv_group[key][()], rf_group[key][()])

        true_vertices = hf['simulation_parameters/true_asteroid/vertices'][()]
        true_faces = hf['simulation_parameters/true_asteroid/faces'][()]
        true_volume = stats.volume(true_vertices, true_faces)

    publication.plot_volume(t_array, vol_array, true_volume)

def landing_site_plots(filename):
    """Given the exploration reconstruction data (after all the exploration)
    
    This function will select a specific area and generate surface slope/roughness 
    plots
    """

    # generate a surface slope map for each face of an asteroid
    # load a asteroid
    vertices, faces = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj');
    # compute radius of each vertex and lat/long 
    spherical_vertices = wavefront.cartesian2spherical(vertices)
    r = spherical_vertices[:, 0]
    lat = spherical_vertices[:, 1]
    long = spherical_vertices[:, 2]
    # interpolate and create a surface plot/contour
    fig, ax = plt.subplots(1, 1)
    grid_long, grid_lat = np.mgrid[-np.pi:np.pi:100j, -np.pi/2:np.pi/2:100j]
    grid_z0 = interpolate.griddata(np.vstack((long, lat)).T, r, (grid_long, grid_lat), method='nearest')
    ax.scatter(long, lat,c=r)
    # ax.contour(grid_long, grid_lat, grid_z0)
    ax.imshow(grid_z0.T, extent=(-np.pi, np.pi, -np.pi/2, np.pi/2), origin='lower')
    plt.show()

if __name__ == "__main__":
    logging_file = tempfile.mkstemp(suffix='.txt.')[1]
    # logging_file = "/tmp/exploration_log.txt"

    logging.basicConfig(filename=logging_file,
                        filemode='w', level=logging.INFO,
                        format='%(asctime)s %(levelname)-8s %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    print("Logging to {}".format(logging_file))

    parser = argparse.ArgumentParser(description="Exploration and asteroid reconstruction simulation",
                                      formatter_class=argparse.RawTextHelpFormatter)
    
    parser.add_argument("simulation_data",
                        help="Filename to store the simulation data",
                        action="store")
    # parser.add_argument("reconstruct_data",
    #                     help="Filename to store the reconstruction data")
    parser.add_argument("-mc", "--move_cam", help="For use with the -a, --animate option. This will translate the camera and give you a view from the satellite",
                        action="store_true")
    parser.add_argument("-mw", "--mesh_weight", help="For use with the -a, --animate option. This will add the uncertainty as a colormap to the asteroid",
                        action="store_true")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("-s", "--simulate", help="Run the exploration simulation",
                       action="store_true")
    group.add_argument("-c", "--control_sim", help="Exploration with a control cost component",
                       action="store_true")
    group.add_argument("-a", "--animate", help="Animate the data from the exploration sim",
                       action="store_true")
    group.add_argument("-r", "--reconstruct", help="Generate images for the reconstruction",
                       action="store_true")
    group.add_argument("-u", "--uncertainty", help="Generate uncertainty plot",
                       action="store_true")
    group.add_argument("-au", "--animate_uncertainty", help="Animate map view of uncertainty over time",
                       action="store_true")
    group.add_argument("-st", "--state" , help="Generate state trajectory plots",
                       action="store_true")
    group.add_argument("-v", "--volume", help="Generate plot of volume",
                       action="store_true")
    group.add_argument("-sa", "--save_animation", help="Save the animation as a sequence of images",
                       action="store_true")
    group.add_argument("-l" , "--landing", help="Continue from the end of exploration to the surface",
                       nargs=1, action="store")
    group.add_argument("-la", "--landing_animation", help="Landing animation",
                       nargs=1, action="store")
    group.add_argument("-lsa", "--landing_save_animation", help="Save landing animation to a video",
                       action="store", nargs=1)

    args = parser.parse_args()
                                                                
    if args.simulate:
        simulate(args.simulation_data)
    elif args.control_sim:
        simulate_control(args.simulation_data)
    elif args.reconstruct:
        output_path = tempfile.mkdtemp()
        reconstruct_images(args.simulation_data, output_path)
        print("Images saved to: {}".format(output_path))
    elif args.animate:
        animate(args.simulation_data, move_cam=args.move_cam,
                mesh_weight=args.mesh_weight)
    elif args.uncertainty:
        plot_uncertainty(args.simulation_data)
    elif args.state:
        plot_state_trajectory(args.simulation_data)
    elif args.volume:
        plot_volume(args.simulation_data)
    elif args.animate_uncertainty:
        animate_uncertainty(args.simulation_data)
    elif args.save_animation:
        save_animation(args.simulation_data, move_cam=args.move_cam,
                       mesh_weight=args.mesh_weight)
    elif args.landing:
        landing(args.landing[0], args.simulation_data)
    elif args.landing_animation:
        animate_landing(args.landing_animation[0], move_cam=args.move_cam, mesh_weight=args.mesh_weight)
    elif args.landing_save_animation:
        save_animate_landing(args.landing_save_animation[0], move_cam=args.move_cam, mesh_weight=args.mesh_weight) 

