"""Module to test out some wavefront function with plots
"""
import pdb
import numpy as np

from point_cloud import wavefront
from visualization import graphics
from dynamics import asteroid

def test_normal_face_plot():
    """Plot the normals to the faces on a mesh and view in Mayavi
    """

    v, f = wavefront.read_obj('./integration/cube.obj')
    normal_face = wavefront.normal_face(v, f)
    cof = wavefront.center_of_face(v, f)

    mfig = graphics.mayavi_figure()
    _ = graphics.mayavi_addMesh(mfig, v, f)
    
    for p1, u in zip(cof, normal_face):
        graphics.mayavi_addPoint(mfig, p1, radius=0.1, color=(1, 0, 0))
        graphics.mayavi_addLine(mfig, p1, u + p1)

def test_closest_edge_plot():
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')
    

    pt = np.array([0.5, 0, 0.1])
    D, P, F, V = wavefront.distance_to_edges(pt, v, f, 
                                             ast.asteroid_grav['normal_face'])

    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_addPoint(mfig, P, radius=0.1, color=(1, 0, 0))
    
    # different color for each face
    for f_ind in F:
        face_verts = v[f[f_ind,:],:]
        graphics.mayavi_addMesh(mfig, face_verts,[(0, 1, 2)], color=tuple(np.random.rand(3)))
    pass

def test_closest_vertex_plot_asteroid():
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v =  ast.asteroid_grav['V']
    f = ast.asteroid_grav['F']
    pt = np.array([1, 0, 0])
    D, P, F, V = wavefront.distance_to_vertices(pt, v, f, 
                                                ast.asteroid_grav['normal_face'])

    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_addPoint(mfig, P, radius=0.1, color=(1, 0, 0))
    
    # different color for each face
    for f_ind in F:
        face_verts = v[f[f_ind,:],:]
        graphics.mayavi_addMesh(mfig, face_verts,[(0, 1, 2)], color=tuple(np.random.rand(3)))

def test_closest_vertex_plot_cube():
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')
    pt = np.array([2, 0, 0])
    D, P, F, V = wavefront.distance_to_vertices(pt, v, f, 
                                                ast.asteroid_grav['normal_face'])

    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_addPoint(mfig, P, radius=0.1, color=(1, 0, 0))
    
    # different color for each face
    for f_ind in F:
        face_verts = v[f[f_ind,:],:]
        graphics.mayavi_addMesh(mfig, face_verts,[(0, 1, 2)], color=tuple(np.random.rand(3)))

# TODO Add a funciton to plot pt, and closest vertex, edge, and face in a color
if __name__ == "__main__":
    test_closest_edge_plot()
    # test_closest_vertex_plot_cube()
    # test_closest_vertex_plot_asteroid()

    # test_normal_face_plot()


