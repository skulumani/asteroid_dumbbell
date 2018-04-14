"""Simulation of a spacecraft with a LIDAR taking measurements 
around an asteroid
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import pdb
import logging
from collections import defaultdict
import os
import tempfile
import argparse
import subprocess
import itertools

import warnings
warnings.simplefilter(action="ignore", category=FutureWarning)

import numpy as np
from scipy import integrate
import h5py

from dynamics import asteroid, dumbbell, eoms, controller
from kinematics import attitude
from visualization import plotting, graphics, animation
from point_cloud import wavefront, raycaster
import utilities
from lib import surface_mesh

# simulate dumbbell moving aroudn asteroid


def initialize():
    """Initialize all the things for the simulation
    """
    logger = logging.getLogger(__name__)
    logger.info('Initialize asteroid and dumbbell objects')

    ast = asteroid.Asteroid('castalia', 4092, 'obj')
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
    des_att_func = controller.random_sweep_attitude
    des_tran_func = controller.inertial_fixed_state
    AbsTol = 1e-9
    RelTol = 1e-9

    return ast, dum, des_att_func, des_tran_func, AbsTol, RelTol


def simulate():

    logger = logging.getLogger(__name__)

    ast, dum, des_att_func, des_tran_func, AbsTol, RelTol = initialize()

    num_steps = int(1e3)
    time = np.linspace(0, num_steps, num_steps)
    t0, tf = time[0], time[-1]
    dt = time[1] - time[0]

    initial_pos = np.array([1.5, 0, 0])
    initial_vel = np.array([0, 0, 0])
    initial_R = attitude.rot3(np.pi / 2).reshape(-1)
    initial_w = np.array([0, 0, 0])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

    # TODO Initialize a coarse asteroid mesh model and combine with piont cloud data

    # initialize the raycaster and lidar
    polydata = wavefront.meshtopolydata(ast.V, ast.F)
    caster = raycaster.RayCaster(polydata)
    sensor = raycaster.Lidar(dist=5)
    # try both a controlled and uncontrolled simulation
    # t, istate, astate, bstate = eoms.inertial_eoms_driver(initial_state, time, ast, dum)

    # TODO Dynamics should be based on the course model
    # TODO Asteroid class will need a method to update mesh
    system = integrate.ode(eoms.eoms_controlled_inertial)
    system.set_integrator('lsoda', atol=AbsTol, rtol=RelTol, nsteps=1e4)
    system.set_initial_value(initial_state, t0)
    system.set_f_params(ast, dum, des_att_func, des_tran_func)

    point_cloud = defaultdict(list)

    state = np.zeros((num_steps + 1, 18))
    t = np.zeros(num_steps + 1)
    int_array = []
    state[0, :] = initial_state

    ii = 1
    while system.successful() and system.t < tf:

        # integrate the system and save state to an array
        t[ii] = (system.t + dt)
        state[ii, :] = system.integrate(system.t + dt)

        logger.info('Step : {} Time: {}'.format(ii, t[ii]))
        # now do the raycasting
        if not (np.floor(t[ii]) % 1):
            logger.info('Raycasting at t = {}'.format(t[ii]))

            targets = sensor.define_targets(state[ii, 0:3],
                                            state[ii, 6:15].reshape((3, 3)),
                                            np.linalg.norm(state[ii, 0:3]))

            # new asteroid rotation with vertices
            nv = ast.rotate_vertices(t[ii])
            Ra = ast.rot_ast2int(t[ii])
            # update the mesh inside the caster
            caster = raycaster.RayCaster.updatemesh(nv, ast.F)

            # these intersections are points in the inertial frame
            intersections = caster.castarray(state[ii, 0:3], targets)

            point_cloud['time'].append(t[ii])
            point_cloud['ast_state'].append(Ra.reshape(-1))
            point_cloud['sc_state'].append(state[ii, :])
            point_cloud['targets'].append(targets)
            point_cloud['inertial_ints'].append(intersections)

            logger.info('Found {} intersections'.format(len(intersections)))

            ast_ints = []
            for pt in intersections:
                if pt.size > 0:
                    pt_ast = Ra.T.dot(pt)
                else:
                    logger.info('No intersection for this point')
                    pt_ast = np.array([np.nan, np.nan, np.nan])

                ast_ints.append(pt_ast)

            point_cloud['ast_ints'].append(np.asarray(ast_ints))

        # TODO Eventually call the surface reconstruction function and update asteroid model

        # create an asteroid and dumbbell
        ii += 1

    # plot the simulation
    # plotting.animate_inertial_trajectory(t, istate, ast, dum)
    # plotting.plot_controlled_inertial(t, istate, ast, dum, fwidth=1)

    return time, state, point_cloud

def kinematics_simulation():
    """Test out a  controlled reconstruction with moving to new pionts"""

    logger = logging.getLogger(__name__)
    # define a new asteroid and reconstructor

    ast, dum, _, _, _, _ = initialize()


def animate(time, state, ast, dum, point_cloud):
    graphics.point_cloud_asteroid_frame(point_cloud)

    mfig = graphics.mayavi_figure(size=(800, 600))
    mesh, ast_axes = graphics.draw_polyhedron_mayavi(ast.V, ast.F, mfig)

    com, dum_axes = graphics.draw_dumbbell_mayavi(state[0, :], dum, mfig)

    pc_lines = [graphics.mayavi_addLine(
        mfig, state[0, 0:3], p) for p in point_cloud['inertial_ints'][0]]

    animation.inertial_asteroid_trajectory(time, state, ast, dum, point_cloud,
                                           (mesh, ast_axes, com, dum_axes,
                                            pc_lines))


def incremental_reconstruction(input_filename, output_filename, asteroid_name='castalia'):
    """Incrementally update the mesh

    Now we'll use the radial mesh reconstruction.

    """
    logger = logging.getLogger(__name__)
    # output_filename = './data/raycasting/20180226_castalia_reconstruct_highres_45deg_cone.hdf5'
    
    logger.info('Loading {}'.format(input_filename))
    data = np.load(input_filename)
    point_cloud = data['point_cloud'][()]

    # define the asteroid and dumbbell objects

    asteroid_faces = 0
    asteroid_type = 'obj'
    m1, m2, l = 500, 500, 0.003

    ellipsoid_min_angle = 10
    ellipsoid_max_radius = 0.03
    ellipsoid_max_distance = 0.5

    surf_area = 0.01

    ast = asteroid.Asteroid(asteroid_name, asteroid_faces, asteroid_type)
    dum = dumbbell.Dumbbell(m1=m1, m2=m2, l=l)
    
    logger.info('Creating ellipsoid mesh')
    # define a simple mesh to start
    ellipsoid = surface_mesh.SurfMesh(ast.axes[0], ast.axes[1], ast.axes[2],
                                      ellipsoid_min_angle, ellipsoid_max_radius, ellipsoid_max_distance)
    v_est, f_est = ellipsoid.verts(), ellipsoid.faces()
    
    vert_weight = np.full(v_est.shape[0], (np.pi * np.max(ast.axes))**2)
    
    max_angle = wavefront.spherical_surface_area(np.max(ast.axes), surf_area)

    # extract out all the points in the asteroid frame
    time = point_cloud['time'][::1]
    ast_ints = point_cloud['ast_ints'][::1]
    logger.info('Create HDF5 file {}'.format(output_filename))
    with h5py.File(output_filename, 'w') as fout:
        # store some extra data about teh simulation
        v_group = fout.create_group('reconstructed_vertex')
        f_group = fout.create_group('reconstructed_face')
        w_group = fout.create_group('reconstructed_weight')

        sim_data = fout.create_group('simulation_data')
        sim_data.attrs['asteroid_name'] = np.string_(asteroid_name)
        sim_data.attrs['asteroid_faces'] =asteroid_faces
        sim_data.attrs['asteroid_type'] = np.string_(asteroid_type)
        sim_data.attrs['m1'] = dum.m1
        sim_data.attrs['m2'] = dum.m2
        sim_data.attrs['l'] = dum.l

        sim_data.attrs['ellipsoid_axes'] = ast.axes
        sim_data.attrs['ellipsoid_min_angle'] = ellipsoid_min_angle
        sim_data.attrs['ellipsoid_max_radius'] = ellipsoid_max_radius
        sim_data.attrs['ellipsoid_max_distance'] = ellipsoid_max_distance
        sim_data.attrs['surf_area'] = surf_area

        sim_data.attrs['max_angle'] = max_angle
        
        fout.create_dataset('truth_vertex', data=ast.V)
        fout.create_dataset('truth_faces', data=ast.F)
        fout.create_dataset('estimate_faces', data=f_est)

        fout.create_dataset('initial_vertex', data=v_est)
        fout.create_dataset('initial_faces', data=f_est)
        fout.create_dataset('initial_weight', data=vert_weight)

        logger.info('Starting loop over point cloud')
        for ii, (t, points) in enumerate(zip(time, ast_ints)):
            # check if points is empty
            logger.info('Current : t = {} with {} points'.format(t, len(points)))
            for pt in points:
                # incremental update for each point in points
                # check to make sure each pt is not nan
                if not np.any(np.isnan(pt)):
                    v_est, vert_weight = wavefront.spherical_incremental_mesh_update(pt, 
                                                                                     v_est, f_est,
                                                                                     vertex_weight=vert_weight,
                                                                                     max_angle=max_angle)

            # use HD5PY instead
            # save every so often and delete v_array,f_array to save memory
            if (ii % 1) == 0:
                logger.info('Saving data to HDF5. ii = {}, t = {}'.format(ii, t))
                v_group.create_dataset(str(ii), data=v_est)
                f_group.create_dataset(str(ii), data=f_est)
                w_group.create_dataset(str(ii), data=vert_weight)
        
    logger.info('Completed the reconstruction')

    return 0


def read_mesh_reconstruct(filename, output_path='/tmp/reconstruct_images'):
    """Use H5PY to read the data back and plot
    """
    logger = logging.getLogger(__name__)
    logger.info('Starting the image generation')

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
        
        v_initial = hf['initial_vertex'][()]
        f_initial = hf['initial_faces'][()]
        w_initial = hf['initial_weight'][()]

        """Partial images during the reconstruction"""
        logger.info('Starting on partial reconstruction images')

        mfig = graphics.mayavi_figure(offscreen=True)
        mesh = graphics.mayavi_addMesh(mfig, v_initial, f_initial)
        ms = mesh.mlab_source
        graphics.mayavi_axes(mfig, [-1, 1, -1, 1, -1, 1], line_width=5, color=(1, 0, 0))
        graphics.mayavi_view(fig=mfig)

        partial_index = np.array([0, v_keys.shape[0]*1/4, v_keys.shape[0]*1/2,
                                  v_keys.shape[0]*3/4, v_keys.shape[0]*4/4-1],
                                 dtype=np.int)
        for img_index, vk in enumerate(partial_index):
            filename = os.path.join(output_path, 'partial_' + str(vk) + '.jpg')
            v = rv[str(vk)][()]
            # generate an image and save it 
            ms.reset(x=v[:, 0], y=v[:, 1], z=v[:,2], triangles=f_initial)
            graphics.mlab.savefig(filename, magnification=4)
        
        """Partial images using a colormap for the data"""
        logger.info('Now using a colormap for the uncertainty')
        mfig = graphics.mayavi_figure(offscreen=True)
        mesh = graphics.mayavi_addMesh(mfig, v_initial, f_initial,
                                       color=None, colormap='viridis',
                                       scalars=w_initial)
        ms = mesh.mlab_source
        graphics.mayavi_axes(mfig, [-1, 1, -1, 1, -1, 1], line_width=5, color=(1, 0, 0))
        graphics.mayavi_view(fig=mfig)

        partial_index = np.array([0, v_keys.shape[0]*1/4, v_keys.shape[0]*1/2,
                                  v_keys.shape[0]*3/4, v_keys.shape[0]*4/4-1],
                                 dtype=np.int)
        for img_index, vk in enumerate(partial_index):
            filename = os.path.join(output_path, 'partial_weights_' + str(vk) + '.jpg')
            v = rv[str(vk)][()]
            w = rw[str(vk)][()]
            # generate an image and save it 
            ms.reset(x=v[:, 0], y=v[:, 1], z=v[:,2], triangles=f_initial,
                     scalars=w)
            graphics.mlab.savefig(filename, magnification=4)

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
            graphics.mlab.savefig(filename, magnification=4)

        """Create a bunch of images for animation"""
        logger.info('Now making images for a movie')
        animation_path = os.path.join(output_path, 'animation')
        if not os.path.exists(animation_path):
            os.makedirs(animation_path)
        
        ms.reset(x=v_initial[:, 0], y=v_initial[:, 1], z=v_initial[:, 2], triangles=f_initial)

        for ii, vk in enumerate(v_keys):
            filename = os.path.join(animation_path, str(ii).zfill(7) + '.jpg')
            v = rv[vk][()]
            ms.reset(x=v[:, 0], y=v[:, 1], z=v[:, 2], triangles=f_initial)
            graphics.mayavi_savefig(mfig, filename, magnification=4)
    
    logger.info('Finished')

    return mfig

if __name__ == "__main__":
    # TODO Measure time for run
    logging_file = tempfile.mkstemp(suffix='.txt')[1]
    output_path = tempfile.mkdtemp()

    logging.basicConfig(filename=logging_file,
                        filemode='w', level=logging.INFO,
                        format='%(asctime)s %(levelname)-8s %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    print("Logging to {}".format(logging_file))

    parser = argparse.ArgumentParser(description="Raycasting and Reconstruction simulation",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("-i", '--point_cloud_data', 
                        help="Filename for point cloud data.\n"
                        "This holds the simulation data as an npz")
    parser.add_argument("-o", '--reconstruct_data', 
                        help="Filename for reconstruction data.\n"
                        "This holds the reconstructed data")
    
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-s", "--simulate", help='Run the point cloud simulation',
                        action="store_true")
    # reconstruct has several different options based on the selection
    group.add_argument("-r", "--reconstruct",
                       help="Run the reconstruction algorithm.\n",
                       action="store_true")
    # plot the output - 1 matches r 1 and 2 matches with r 2
    group.add_argument("-p", "--plot",
                       help="Read the reconstruction data and images\n",
                       action="store_true")
    group.add_argument("-m", "--movie", nargs=2,
                       help="Create movie from all images in temp folder\n"
                       "Path to images folder\n"
                       "Filename to parse by ffmpeg, i.e picture_<percent>06d.jpg",
                       action="append")

    args = parser.parse_args()
    
    if args.simulate:
        time, state, point_cloud = simulate()

        # save data to a file
        np.savez(args.point_cloud_data, time=time, state=state,
                 point_cloud=point_cloud)

        # to access the data again
        # data = np.load(filename)
        # point_cloud = data['point_cloud'][()]
    elif args.reconstruct:
        # now reconstruct
        # reconstruct_filename = os.path.join('./data/raycasting', args.fnames[1])
        # filename = './data/raycasting/20180110_raycasting_castalia.npz'
        
        incremental_reconstruction(args.point_cloud_data, args.reconstruct_data, 'castalia')
    elif args.plot:
        # generate the images
        print("Images saved to {}".format(output_path))
        read_mesh_reconstruct(args.reconstruct_data, output_path=output_path)
    elif args.movie[0]:
        os.chdir(args.movie[0][0])
        subprocess.call(['ffmpeg', '-i', args.movie[0][1], 'output.mp4'])
        print("Movie is created in {}/output.mp4".format(args.movie[0][0]))
    # also automatically create the video by calling ffmpeg
    # print("Now going to create a video using FFMPEG")
    # ffmpeg_command ='ffmpeg -framerate -i %06d.jpg' + " -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p -vf \"scale=trunc(iw/2)*2:trunc(ih/2)*2\" video.mp4"
    # subprocess.run(ffmpeg_command, shell=True, cwd=output_path)

