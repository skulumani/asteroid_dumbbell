# Visualize using Mayavi
from __future__ import absolute_import, division, print_function, unicode_literals
from mayavi import mlab
import numpy as np 
import dynamics.asteroid as asteroid
from kinematics import attitude
from visualization import graphics
import pdb

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

@mlab.animate()
def inertial_asteroid_trajectory(time, state, ast, dum,
                                 mayavi_objects):
    """Animate the rotation of an asteroid and the motion of SC
    """
    mesh, com = mayavi_objects
    # animate the rotation fo the asteroid
    ms = mesh.mlab_source
    ts = com.mlab_source

    for (t, pos) in zip(time, state[:, 0:3]):
        # rotate teh asteroid
        Ra = attitude.rot3(ast.omega * t, 'c')
        # parse out the vertices x, y, z
        new_vertices = Ra.dot(ast.V.T).T
    
        # update asteroid
        ms.set(x=new_vertices[:, 0],y=new_vertices[:, 1], z=new_vertices[:,2])
        # update the satellite
        ts.set(x=pos[0], y=pos[1], z=pos[2])
        yield
