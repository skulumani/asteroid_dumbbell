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

import h5py
import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt

from lib import asteroid, surface_mesh, cgal, mesh_data, reconstruct
from lib import surface_mesh
from lib import controller as controller_cpp

from dynamics import dumbbell, eoms, controller
from point_cloud import wavefront
from kinematics import attitude
import utilities
from visualization import graphics, animation, publication

compression = 'gzip'
compression_opts = 9
max_steps = 15000

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

            logger.info("Step: {} Time: {}".format(ii, t))

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
        mesh, ast_axes = graphics.draw_polyhedron_mayavi(est_initial_vertices, est_initial_faces, mfig)

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
    graphics.mlab.show()

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

        """Create a bunch of images for animation"""
        logger.info('Now making images for a movie')
        animation_path = os.path.join(output_path, 'animation')
        if not os.path.exists(animation_path):
            os.makedirs(animation_path)
        
        ms.set(x=v_initial[:, 0], y=v_initial[:, 1], z=v_initial[:, 2], triangles=f_initial)

        for ii, vk in enumerate(v_keys):
            filename = os.path.join(animation_path, str(ii).zfill(7) + '.jpg')
            v = rv[vk][()]
            ms.reset(x=v[:, 0], y=v[:, 1], z=v[:, 2], triangles=f_initial)
            graphics.mayavi_savefig(mfig, filename, magnification=magnification)
    

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
                        help="Filename to store the simulation data")
    # parser.add_argument("reconstruct_data",
    #                     help="Filename to store the reconstruction data")

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
        animate(args.simulation_data)
    elif args.uncertainty:
        plot_uncertainty(args.simulation_data)


