# Visualize using Mayavi
from __future__ import absolute_import, division, print_function, unicode_literals

from mayavi import mlab
import h5py
import numpy as np 
import pdb

import dynamics.asteroid as asteroid
from kinematics import attitude
from visualization import graphics
import utilities


def test_asteroid():
    # plot the asteroid as a triangular mesh

    ast = asteroid.Asteroid('castalia', 64)

    V = ast.asteroid_grav.get('V')*1e5
    F = ast.asteroid_grav.get('F')
    F = F.astype(int)
    surface = mlab.pipeline.triangular_mesh_source(V[:,0], V[:,1], V[:,2], F-1)
    # numpy array of points which define the vertices of the surface
    points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0]])
    # numpy array defining the triangle which connects those points
    element = np.array([[0, 1, 2], [0, 1, 2]])

    # mlab surface defined with the points and element
    # surface = mlab.triangular_mesh(points[:, 0], points[:, 1], points[:, 2], element)

    # mlab.show()
    # mlab.triangular_mesh(V[:,0], V[:,1], V[:,2], F)

    # mlab.show()

def test_mesh():
    """A very pretty picture of spherical harmonics translated from
    the octaviz example."""
    pi = np.pi
    cos = np.cos
    sin = np.sin
    dphi, dtheta = np.pi / 250.0, np.pi / 250.0
    [phi, theta] = np.mgrid[0:pi + dphi * 1.5:dphi,
                               0:2 * pi + dtheta * 1.5:dtheta]
    m0 = 4
    m1 = 3
    m2 = 2
    m3 = 3
    m4 = 6
    m5 = 2
    m6 = 6
    m7 = 4

    r = np.sin(m0 * phi) ** m1 + np.cos(m2 * phi) ** m3 + \
        np.sin(m4 * theta) ** m5 + np.cos(m6 * theta) ** m7
    x = r * np.sin(phi) * np.cos(theta)
    y = r * np.cos(phi)
    z = r * np.sin(phi) * np.sin(theta)

    return mlab.mesh(x, y, z, colormap="bone")

def test_triangular_mesh():
    """An example of a cone, ie a non-regular mesh defined by its
        triangles.
    """
    n = 8
    t = np.linspace(-np.pi, np.pi, n)
    z = np.exp(1j * t)
    x = z.real.copy()
    y = z.imag.copy()
    z = np.zeros_like(x)

    triangles = [(0, i, i + 1) for i in range(1, n)]
    x = np.r_[0, x]
    y = np.r_[0, y]
    z = np.r_[1, z]
    t = np.r_[0, t]

    return mlab.triangular_mesh(x, y, z, triangles)

def update_axes(axes_tuple, com, Rb2i):
    """Update the axes given a rotation matrix
    """
    axes_source = (axis.mlab_source for axis in axes_tuple)
    
    for ii, axis_source in enumerate(axes_source):
        axis_source.set(x=[com[0], Rb2i[0, ii]], y=[com[1], Rb2i[1, ii]],
                        z=[com[2], Rb2i[2, ii]])

@mlab.animate()
def inertial_asteroid_trajectory(time, state, ast, dum, point_cloud, 
                                 mayavi_objects):
    """Animate the rotation of an asteroid and the motion of SC
    """
    mesh, ast_axes, com, dum_axes, pc_lines = mayavi_objects

    # animate the rotation fo the asteroid
    ms = mesh.mlab_source
    ts = com.mlab_source

    ast_xs = ast_axes[0].mlab_source
    ast_ys = ast_axes[1].mlab_source
    ast_zs = ast_axes[2].mlab_source

    dum_xs = dum_axes[0].mlab_source
    dum_ys = dum_axes[1].mlab_source
    dum_zs = dum_axes[2].mlab_source
    
    pc_sources = [line.mlab_source for line in pc_lines]

    for (t, pos, Rb2i, intersections) in zip(time, state[:, 0:3], state[:, 6:15],
                                             point_cloud['inertial_ints']):
        # rotate teh asteroid
        Ra = attitude.rot3(ast.omega * t, 'c')
        Rb2i = Rb2i.reshape((3,3))
        # parse out the vertices x, y, z
        new_vertices = ast.rotate_vertices(t)
    
        # update asteroid
        ms.set(x=new_vertices[:, 0],y=new_vertices[:, 1], z=new_vertices[:,2])
        # update the satellite
        ts.set(x=pos[0], y=pos[1], z=pos[2])
        
        # update the asteroid axes
        ast_xs.reset(x=[0, Ra[0,0]], y=[0,Ra[1,0]], z=[0, Ra[2,0]])
        ast_ys.reset(x=[0, Ra[0,1]], y=[0, Ra[1,1]], z=[0, Ra[2,1]])
        ast_zs.reset(x=[0, Ra[0,2]], y=[0, Ra[1,2]], z=[0, Ra[2,2]])

        dum_xs.reset(x=[pos[0], pos[0]+Rb2i[0,0]], y=[pos[1], pos[1]+Rb2i[1,0]],
                     z=[pos[2], pos[2]+Rb2i[2,0]])
        dum_ys.reset(x=[pos[0], pos[0]+Rb2i[0,1]], y=[pos[1], pos[1]+Rb2i[1,1]],
                     z=[pos[2], pos[2]+Rb2i[2,1]])
        dum_zs.reset(x=[pos[0], pos[0]+Rb2i[0,2]], y=[pos[1], pos[1]+Rb2i[1,2]],
                     z=[pos[2], pos[2]+Rb2i[2,2]])
        
        for pcs, inter in zip(pc_sources, intersections):
            # check if intersection is empty
            if inter.size > 2:
                pcs.reset(x=[pos[0], inter[0]], y=[pos[1], inter[1]], z=[pos[2], inter[2]])
            else:
                pcs.reset(x=[pos[0], pos[0]+0.01], y=[pos[1], pos[1]+0.01], z=[pos[2], pos[2]+0.01])
        yield

@mlab.animate()
def inertial_asteroid_trajectory_cpp(time, state, inertial_intersections,
                                     hdf5_file, mayavi_objects):
    """Animate the rotation of an asteroid and the motion of SC

    This assumes an asteroid object from C++ and using the exploration sim
    """
    mesh, ast_axes, com, dum_axes, pc_lines = mayavi_objects
    
    # get all the keys for the reconstructed vertices and faces

    # animate the rotation fo the asteroid
    ms = mesh.mlab_source
    ts = com.mlab_source

    ast_xs = ast_axes[0].mlab_source
    ast_ys = ast_axes[1].mlab_source
    ast_zs = ast_axes[2].mlab_source

    dum_xs = dum_axes[0].mlab_source
    dum_ys = dum_axes[1].mlab_source
    dum_zs = dum_axes[2].mlab_source
    
    pc_sources = [line.mlab_source for line in pc_lines]
    with h5py.File(hdf5_file, 'r') as hf:
        rv_group = hf['reconstructed_vertex']
        rf_group = hf['reconstructed_face']
        Ra_group = hf['Ra']

        rv_keys = np.array(utilities.sorted_nicely(list(rv_group.keys())))
        
        for (t, pos, Rb2i, ints, key) in zip(time, state[:, 0:3], state[:, 6:15],
                                            inertial_intersections,
                                            rv_keys):
            # rotate teh asteroid
            # Ra = ast.rot_ast2int(t)
            Ra = Ra_group[key][()]
            Rb2i = Rb2i.reshape((3,3))
            # parse out the vertices x, y, z
            # rotate the asteroid
            new_vertices = Ra.dot(rv_group[key][()].T).T
            new_faces = rf_group[key][()]
            
            # update asteroid
            ms.reset(x=new_vertices[:, 0],y=new_vertices[:, 1], z=new_vertices[:,2],
                    triangles=new_faces)
            # update the satellite
            ts.set(x=pos[0], y=pos[1], z=pos[2])
            
            # update the asteroid axes
            ast_xs.reset(x=[0, Ra[0,0]], y=[0,Ra[1,0]], z=[0, Ra[2,0]])
            ast_ys.reset(x=[0, Ra[0,1]], y=[0, Ra[1,1]], z=[0, Ra[2,1]])
            ast_zs.reset(x=[0, Ra[0,2]], y=[0, Ra[1,2]], z=[0, Ra[2,2]])

            dum_xs.reset(x=[pos[0], pos[0]+Rb2i[0,0]], y=[pos[1], pos[1]+Rb2i[1,0]],
                        z=[pos[2], pos[2]+Rb2i[2,0]])
            dum_ys.reset(x=[pos[0], pos[0]+Rb2i[0,1]], y=[pos[1], pos[1]+Rb2i[1,1]],
                        z=[pos[2], pos[2]+Rb2i[2,1]])
            dum_zs.reset(x=[pos[0], pos[0]+Rb2i[0,2]], y=[pos[1], pos[1]+Rb2i[1,2]],
                        z=[pos[2], pos[2]+Rb2i[2,2]])
            
            for pcs, inter in zip(pc_sources, ints):
                # check if intersection is empty
                if inter.size > 2:
                    pcs.reset(x=[pos[0], inter[0]], y=[pos[1], inter[1]], z=[pos[2], inter[2]])
                else:
                    pcs.reset(x=[pos[0], pos[0]+0.01], y=[pos[1], pos[1]+0.01], z=[pos[2], pos[2]+0.01])
            yield
