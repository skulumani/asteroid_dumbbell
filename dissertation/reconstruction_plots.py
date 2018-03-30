"""Plots for reconstruction of a mesh from point cloud
"""
import pdb
import numpy as np
import warnings
import os
import itertools
import scipy.io
import h5py

from point_cloud import wavefront
from visualization import graphics
from kinematics import sphere
from dynamics import asteroid
from lib import surface_mesh
import utilities

view = {'azimuth': 16.944197132093564, 'elevation': 66.34177792039738,
        'distance': 2.9356815748114435, 
        'focalpoint': np.array([0.20105769, -0.00420018, -0.016934045])}

def cube_into_sphere(img_path):
    """Transform a cube into a sphere
    """
    vc, fc = wavefront.read_obj('./integration/cube.obj')
    vs, fs = wavefront.ellipsoid_mesh(2, 2, 2, density=10, subdivisions=0)
    
    
    mfig = graphics.mayavi_figure(offscreen=True)
    mesh = graphics.mayavi_addMesh(mfig, vc,fc)
    ms = mesh.mlab_source
    index = 0
    for ii in range(5):
        for jj, pt in enumerate(vs):
            index += 1
            filename = os.path.join(img_path, 'cube_sphere_' + str(index).zfill(6) + '.jpg')
            graphics.mlab.savefig(filename, magnification=4)
            mesh_param = wavefront.polyhedron_parameters(vc, fc)
            vc, fc = wavefront.radius_mesh_incremental_update(pt, vc, fc,
                                                              mesh_param,
                                                              max_angle=np.deg2rad(5))
            ms.reset(x=vc[:, 0], y=vc[:, 1], z=vc[:, 2], triangles=fc)
            graphics.mayavi_addPoint(mfig, pt)
        
        vc, fc = wavefront.mesh_subdivide(vc, fc, 1)
        ms.reset(x=vc[:, 0], y=vc[:, 1], z=vc[:, 2], triangles=fc)

    return 0

def sphere_into_ellipsoid(img_path):
    """See if we can turn a sphere into an ellipse by changing the radius of
    vertices
    
    The point cloud (ellipse) should have more points than the initial mesh.
    When the intial mesh is coarse the resulting mesh will also be heavily faceted, but this will avoid the big holes, and large changes in depth
    """

    # define the sphere
    vs, fs = wavefront.ellipsoid_mesh(1, 1, 1, density=20, subdivisions=1)
    ve, fe = wavefront.ellipsoid_mesh(2, 3, 4, density=20, subdivisions=1)
    
    mfig = graphics.mayavi_figure(offscreen=True)
    mesh = graphics.mayavi_addMesh(mfig, vs, fs)
    ms = mesh.mlab_source
    index = 0
    # in a loop add each vertex of the ellipse into the sphere mesh
    for jj in range(2):
        for ii, pt in enumerate(ve):
            index +=1
            filename = os.path.join(img_path, 'sphere_ellipsoid_' + str(index).zfill(6) + '.jpg')
            graphics.mlab.savefig(filename, magnification=4)
            mesh_param = wavefront.polyhedron_parameters(vs, fs)
            vs, fs = wavefront.radius_mesh_incremental_update(pt, vs,fs,
                                                              mesh_param,
                                                              max_angle=np.deg2rad(10))
            ms.reset(x=vs[:,0], y=vs[:,1], z=vs[:,2], triangles=fs)
            graphics.mayavi_addPoint(mfig, pt)
    
        vs, fs = wavefront.mesh_subdivide(vs, fs,  1)
        ms.reset(x=vs[:,0], y=vs[:,1], z=vs[:,2], triangles=fs)

    return 0

def castalia_reconstruction(img_path):
    """Incrementally modify an ellipse into a low resolution verision of castalia
    by adding vertices and modifying the mesh
    """
    surf_area = 0.01
    a = 0.22
    delta = 0.01

    # load a low resolution ellipse to start
    ast = asteroid.Asteroid('castalia', 0, 'obj')
    ellipsoid = surface_mesh.SurfMesh(ast.axes[0], ast.axes[1], ast.axes[2],
                                     10, 0.025, 0.5)
    
    ve, fe = ellipsoid.verts(), ellipsoid.faces()
    vc, fc = ast.V, ast.F

    # sort the vertices in in order (x component)
    vc = vc[vc[:, 0].argsort()]

    # uncertainty for each vertex in meters (1/variance)
    vert_weight = np.full(ve.shape[0], (np.pi*np.max(ast.axes))**2)
    # calculate maximum angle as function of surface area
    max_angle = wavefront.spherical_surface_area(np.max(ast.axes), surf_area)

    # loop and create many figures
    mfig = graphics.mayavi_figure(offscreen=False)
    mesh = graphics.mayavi_addMesh(mfig, ve, fe)
    ms = mesh.mlab_source
    index = 0

    for ii, pt in enumerate(vc):
        index +=1
        filename = os.path.join(img_path, 'castalia_reconstruct_' + str(index).zfill(7) + '.jpg')
        # graphics.mlab.savefig(filename, magnification=4)
        ve, vert_weight = wavefront.spherical_incremental_mesh_update(pt,ve,fe,
                                                                       vertex_weight=vert_weight,
                                                                       max_angle=max_angle)
        
        ms.reset(x=ve[:, 0], y=ve[:, 1], z=ve[:, 2], triangles=fe)
        graphics.mayavi_addPoint(mfig, pt, radius=0.01 )
        
    graphics.mayavi_points3d(mfig,ve, scale_factor=0.01, color=(1, 0, 0))

    return 0

def castalia_reconstruct_generate_data(output_filename):
    """Generate all the data for an example for reconstructing asteroid 
    castalia
    """
    asteroid_name = 'castalia'
    asteroid_type = 'obj'
    asteroid_faces = 0
    
    ellipsoid_min_angle = 10
    ellipsoid_max_radius = 0.03
    ellipsoid_max_distance = 0.5

    surf_area = 0.01
    
    # load asteroid castalia
    ast = asteroid.Asteroid(asteroid_name, asteroid_faces, asteroid_type)
    ellipsoid = surface_mesh.SurfMesh(ast.axes[0], ast.axes[1], ast.axes[2],
                                      ellipsoid_min_angle, ellipsoid_max_radius, ellipsoid_max_distance)
    ve, fe = ellipsoid.verts(), ellipsoid.faces()
    vc, fc = ast.V, ast.F
    
    # cort the truth vertices in increasing order of x component
    vc = vc[vc[:, 0].argsort()]

    # define initial uncertainty for our estimate
    vert_weight = np.full(ve.shape[0], (np.pi * np.max(ast.axes))**2)

    # calculate the maximum angle as a function of desired surface area
    max_angle = wavefront.spherical_surface_area(np.max(ast.axes), surf_area)

    # loop over all the points and save the data
    output_path = os.path.join(output_filename)

    with h5py.File(output_path, 'w') as fout:
        reconstructed_vertex = fout.create_group('reconstructed_vertex')
        reconstructed_weight = fout.create_group('reconstructed_weight')
        reconstructed_vertex.attrs['asteroid_name'] = np.string_(asteroid_name)
        reconstructed_vertex.attrs['asteroid_faces'] = asteroid_faces
        reconstructed_vertex.attrs['asteroid_type'] = np.string_(asteroid_type)
        reconstructed_vertex.attrs['ellipsoid_axes'] = ast.axes
        reconstructed_vertex.attrs['ellipsoid_min_angle'] = ellipsoid_min_angle
        reconstructed_vertex.attrs['ellipsoid_max_radius'] = ellipsoid_max_radius
        reconstructed_vertex.attrs['ellipsoid_max_distance'] = ellipsoid_max_distance
        reconstructed_vertex.attrs['surf_area'] = surf_area

        fout.create_dataset('truth_vertex', data=vc)
        fout.create_dataset('truth_faces', data=fc)
        fout.create_dataset('estimate_faces', data=fe)

        fout.create_dataset('initial_vertex', data=ve)
        fout.create_dataset('initial_faces', data=fe)
        fout.create_dataset('initial_weight', data=vert_weight)

        for ii, pt in enumerate(vc):
            ve, vert_weight = wavefront.spherical_incremental_mesh_update(pt, ve, fe,
                                                                          vertex_weight=vert_weight,
                                                                          max_angle=max_angle)
            # save the current array and weight to htpy
            reconstructed_vertex.create_dataset(str(ii), data=ve)
            reconstructed_weight.create_dataset(str(ii), data=vert_weight)
            
    
    print('Finished generating data. Saved to {}'.format(output_path))
    
    return 0

def castalia_generate_plots(data_path, img_path='/tmp/diss_reconstruct'):
    """Given a HDF5 file this will read the data and create a bunch of plots/images
    """
    # check and create output directory if not existed
    if not os.path.exists(img_path):
        os.makedirs(img_path)

    with h5py.File(data_path, 'r') as hf:
        rv = hf['reconstructed_vertex']
        rw = hf['reconstructed_weight']

        # get all the keys for the groups
        v_keys = np.array(utilities.sorted_nicely(list(rv.keys())))
        w_keys = np.array(utilities.sorted_nicely(list(rw.keys())))
        
        v_initial = hf['initial_vertex'][()]
        f_initial = hf['initial_faces'][()]
    
        # create images at 0%, 25%, 50%, 75% of reconstruction
        mfig = graphics.mayavi_figure(offscreen=True)
        mesh = graphics.mayavi_addMesh(mfig, v_initial, f_initial)
        ms = mesh.mlab_source
        graphics.mlab.savefig(os.path.join(img_path, 'partial_0.jpg'), magnification=4)

        partial_index = np.array([0, v_keys.shape[0]*1/4, v_keys.shape[0]*1/2,
                                  v_keys.shape[0]*3/4, v_keys.shape[0]*4/4-1],
                                 dtype=np.int)
        for img_index, vk in enumerate(partial_index):
            filename = os.path.join(img_path, 'partial_' + str(img_index+1) + '.jpg')
            v = rv[str(vk)][()]
            # generate an image and save it 
            ms.reset(x=v[:, 0], y=v[:, 1], z=v[:,2], triangles=f_initial)
            graphics.mlab.savefig(filename, magnification=4)

    # images at a variety of different angles

    # all the images for use in creating an animation
    
    return 0

def sphere_into_ellipsoid_spherical_coordinates(img_path):
    """See if we can turn a sphere into an ellipse by changing the radius of
    vertices in spherical coordinates
    
    The point cloud (ellipse) should have the same number of points than the initial mesh.
    """
    surf_area = 0.06
    a = 0.25 # at a*100 % of maximum angle the scale will be 50% of measurement
    delta=0.01
    # define the sphere
    # vs, fs = wavefront.ellipsoid_mesh(0.5, 0.5, 0.5, density=10, subdivisions=1)
    # ve, fe = wavefront.ellipsoid_mesh(1,2, 3, density=10, subdivisions=1)
    
    # import the sphere and ellipsoid from matlab files
    sphere_data = scipy.io.loadmat('./data/sphere_distmesh.mat')
    ellipsoid_data = scipy.io.loadmat('./data/ellipsoid_distmesh.mat')
    vs, fs = sphere_data['v'], sphere_data['f']
    ve, fe = ellipsoid_data['v'], ellipsoid_data['f']

    # sphere = surface_mesh.SurfMesh(0.5, 0.5, 0.5, 10, 0.05, 0.5)
    # ellipsoid = surface_mesh.SurfMesh(1, 2, 3, 10, 0.2, 0.5)
    # vs, fs = sphere.verts(), sphere.faces()
    # ve, fe = ellipsoid.verts(), sphere.faces()
    
    print("Sphere V: {} F: {}".format(vs.shape[0], fs.shape[0]))
    print("Ellipsoid V: {} F: {}".format(ve.shape[0], fe.shape[0]))
    # convert to spherical coordinates
    vs_spherical = wavefront.cartesian2spherical(vs)
    ve_spherical = wavefront.cartesian2spherical(ve)

    mfig = graphics.mayavi_figure(offscreen=False)
    mesh = graphics.mayavi_addMesh(mfig, vs, fs)
    ms = mesh.mlab_source
    index = 0

    # graphics.mayavi_points3d(mfig, vs, color=(1, 0, 0))
    # graphics.mayavi_points3d(mfig, ve, color=(0, 1, 0))
    # in a loop add each vertex of the ellipse into the sphere mesh
    for ii, pt in enumerate(ve_spherical):
        index +=1
        filename = os.path.join(img_path, 'sphere_ellipsoid_' + str(index).zfill(6) + '.jpg')
        # graphics.mlab.savefig(filename, magnification=4)
        vs_spherical, fs = wavefront.spherical_incremental_mesh_update(mfig, pt, vs_spherical,fs,
                                                                       surf_area=surf_area,
                                                                       a=a, delta=delta)
        # convert back to cartesian for plotting

        vs_cartesian = wavefront.spherical2cartesian(vs_spherical)
        ms.reset(x=vs_cartesian[:,0], y=vs_cartesian[:,1], z=vs_cartesian[:,2], triangles=fs)
        graphics.mayavi_addPoint(mfig, wavefront.spherical2cartesian(pt), radius=0.02)

    graphics.mayavi_points3d(mfig, vs_cartesian, scale_factor=0.02, color=(1, 0, 0))
    return 0

if __name__ == "__main__":
    img_path = '/tmp/mayavi_figure'
    if not os.path.exists(img_path):
        os.makedirs(img_path)

    # cube_into_sphere(img_path)
    # sphere_into_ellipsoid(img_path)
    castalia_reconstruction(img_path)
    # sphere_into_ellipsoid_spherical_coordinates(img_path)
    # castalia_reconstruction_factor_tuning(img_path)
    # sphere_factor_tuning(img_path)
