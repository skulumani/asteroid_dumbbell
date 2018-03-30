"""Plots for reconstruction of a mesh from point cloud
"""
import pdb
import numpy as np
import warnings
import os
import itertools
import scipy.io

from point_cloud import wavefront
from visualization import graphics
from kinematics import sphere
from dynamics import asteroid
from lib import surface_mesh

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
    ellipsoid = surface_mesh.SurfMesh(ast.axes[0]*0.75, ast.axes[1]*0.75, ast.axes[2]*0.75,
                                     10, 0.02, 0.5)
    
    ve, fe = ellipsoid.verts(), ellipsoid.faces()
    vc, fc = ast.V, ast.F

    # sort the vertices in in order (x component)
    vc = vc[vc[:, 0].argsort()]

    pdb.set_trace()
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
        ve, vert_weight = wavefront.spherical_incremental_mesh_update(mfig, pt,ve,fe,
                                                                       vertex_weight=vert_weight,
                                                                       max_angle=max_angle,
                                                                       a=a, delta=delta)
        
        # back to cartesian
        ms.reset(x=ve[:, 0], y=ve[:, 1], z=ve[:, 2], triangles=fe)
        graphics.mayavi_addPoint(mfig, wavefront.spherical2cartesian(pt), radius=0.01 )
        
    graphics.mayavi_points3d(mfig,ve, scale_factor=0.01, color=(1, 0, 0))

    return 0

def castalia_reconstruction_factor_tuning(img_path):
    """ Vary both the surface area and radius factor to see the effect
    """
    surf_area = np.array([0.005, 0.01])
    radius_cutoff = np.arange(0.25, 0.8, 0.05)
    delta = 0.01

    # load a low resolution ellipse to start
    ast = asteroid.Asteroid('castalia', 0, 'obj')
    ellipsoid = surface_mesh.SurfMesh(ast.axes[0]*0.75, ast.axes[1]*0.75, ast.axes[2]*0.75,
                                     10, 0.02, 0.5)
    
    ve, fe = ellipsoid.verts(), ellipsoid.faces()
    vc, fc = ast.V, ast.F

    # sort the vertices in in order (x component)
    # vc = vc[vc[:, 0].argsort()]

    # both now into spherical coordinates
    ve_spherical = wavefront.cartesian2spherical(ve)
    vc_spherical = wavefront.cartesian2spherical(vc)

    # loop and create many figures
    mfig = graphics.mayavi_figure(offscreen=True)
    mesh = graphics.mayavi_addMesh(mfig, ve, fe)
    ms = mesh.mlab_source
    
    graphics.mayavi_points3d(mfig, vc, scale_factor=0.01)

    for sa, a in itertools.product(surf_area, radius_cutoff):
        index = 0
        filename = os.path.join(img_path, 'castalia_reconstruct_nosort_' + 'sa=' + str(sa).replace('.','') + '_rf=' + str(a).replace('.','') + '.jpg')
        
        # reset
        ve_s = ve_spherical.copy()
        for ii, pt in enumerate(vc_spherical):
            ve_s, fc = wavefront.spherical_incremental_mesh_update(mfig, pt,
                                                                   ve_s, fe,
                                                                   surf_area=sa,
                                                                   a=a, delta=delta)
            
        # back to cartesian
        ve_c = wavefront.spherical2cartesian(ve_s)

        ms.reset(x=ve_c[:, 0], y=ve_c[:, 1], z=ve_c[:, 2], triangles=fc)
        graphics.mayavi_addTitle(mfig, 'Surface Area={} Cutoff={}'.format(sa, a), color=(0, 0, 0))
        graphics.mlab.savefig(filename, magnification=4)

def sphere_factor_tuning(img_path):
    """ Vary both the surface area and radius factor to see the effect
    """
    surf_area = np.array([0.06])
    radius_cutoff = np.array([0.25])
    delta = 0.01

    # load a low resolution ellipse to start
    sphere_small = surface_mesh.SurfMesh(0.5, 0.5, 0.5, 10, 0.05, 0.5)
    sphere_large = surface_mesh.SurfMesh(1, 1, 1, 10, 0.2, 0.5)
    vi, fi = sphere_small.verts(), sphere_small.faces()
    vf, ff = sphere_large.verts(), sphere_large.faces()
    
    print('Initial #V: {}'.format(vi.shape[0]))
    print('Final #V: {}'.format(vf.shape[0]))
    pdb.set_trace()
    # sort the vertices in in order (x component)
    # vc = vc[vc[:, 0].argsort()]

    # both now into spherical coordinates
    vi_spherical = wavefront.cartesian2spherical(vi)
    vf_spherical = wavefront.cartesian2spherical(vf)

    # loop and create many figures
    mfig = graphics.mayavi_figure(offscreen=True)
    mesh = graphics.mayavi_addMesh(mfig, vi, fi)
    ms = mesh.mlab_source
    
    # graphics.mayavi_points3d(mfig, vc, scale_factor=0.01)

    for sa, a in itertools.product(surf_area, radius_cutoff):
        index = 0
        filename = os.path.join(img_path, 'sphere_reconstruct_nosort' + 'sa=' + str(sa).replace('.','') + '_rf=' + str(a).replace('.','') + '.jpg')
        
        # reset
        vi_s = vi_spherical.copy()
        for ii, pt in enumerate(vf_spherical):
            vi_s, fc = wavefront.spherical_incremental_mesh_update(mfig, pt,
                                                                   vi_s, fi,
                                                                   surf_area=sa,
                                                                   a=a, delta=delta)
            
        # back to cartesian
        vi_c = wavefront.spherical2cartesian(vi_s)

        ms.reset(x=vi_c[:, 0], y=vi_c[:, 1], z=vi_c[:, 2], triangles=fc)
        graphics.mayavi_addTitle(mfig, 'Surface Area={} Cutoff={}'.format(sa, a), color=(0, 0, 0))
        graphics.mlab.savefig(filename, magnification=4)

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
