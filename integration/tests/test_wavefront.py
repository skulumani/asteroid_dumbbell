"""Module to test out some wavefront function with plots
"""
import pdb
import numpy as np
import warnings

from point_cloud import wavefront
from visualization import graphics
from dynamics import asteroid
from kinematics import sphere

warnings.filterwarnings(action="ignore", category=FutureWarning,
                        message=r"Conversion")

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

    graphics.mayavi_addTitle(mfig, 'Normals to each face', color=(0, 0, 0), size=0.5)

def test_closest_vertex_plot_cube():
    pt = np.random.uniform(0.6 * np.sqrt(3), 1) * sphere.rand(2)

    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')
    D, P, F, V = wavefront.distance_to_vertices(pt, v, f, 
                                                ast.asteroid_grav['normal_face'],
                                                ast.asteroid_grav['vertex_face_map'])

    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_points3d(mfig, P, scale_factor=0.1, color=(1, 0, 0))
    
    # different color for each face
    for f_ind in F:
        face_verts = v[f[f_ind,:],:]
        graphics.mayavi_addMesh(mfig, face_verts,[(0, 1, 2)], color=tuple(np.random.rand(3)))

    graphics.mayavi_addTitle(mfig, 'Closest Vertex', color=(0, 0, 0), size=0.5)

def test_closest_vertex_plot_asteroid():
    pt = np.random.uniform(1, 1.5) * sphere.rand(2)

    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v =  ast.asteroid_grav['V']
    f = ast.asteroid_grav['F']
    D, P, F, V = wavefront.distance_to_vertices(pt, v, f, 
                                                ast.asteroid_grav['normal_face'],
                                                ast.asteroid_grav['vertex_face_map'])

    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_points3d(mfig, P,scale_factor=0.1, color=(1, 0, 0))
    
    # different color for each face
    for f_ind in F:
        face_verts = v[f[f_ind,:],:]
        graphics.mayavi_addMesh(mfig, face_verts,[(0, 1, 2)], color=tuple(np.random.rand(3)))

    graphics.mayavi_addTitle(mfig, 'Closest Vertex', color=(0, 0, 0), size=0.5)

def test_closest_edge_plot_cube(pt=np.array([0.7, 0, 0])):
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')
    
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, F, V = wavefront.distance_to_edges(pt, v, f, normal_face,
                                             edge_vertex_map, edge_face_map,
                                             vf_map)
    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_points3d(mfig, P, scale_factor=0.1, color=(1, 0, 0))
    
    # different color for each face
    for edge_faces in F:
        for f_ind in edge_faces:
            pdb.set_trace()
            face_verts = v[f[f_ind,:],:]
            graphics.mayavi_addMesh(mfig, face_verts, [(0, 1, 2)], color=tuple(np.random.rand(3)))
    
    # draw the points which make up the edges and draw a line for the edge
    for v_ind in V:
        graphics.mayavi_addPoint(mfig, v[v_ind,:], radius=0.1, color=(0, 0, 1))
    
    graphics.mayavi_addLine(mfig, v[V[0],:], v[V[1],:], color=(0, 0, 1))

    graphics.mayavi_addTitle(mfig, 'Closest Edge', color=(0, 0, 0), size=0.5)

def test_closest_edge_plot_asteroid():
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    
    v = ast.asteroid_grav['V']
    f = ast.asteroid_grav['F']

    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']

    pt = np.array([2, 0, 0])
    D, P, F, V = wavefront.distance_to_edges(pt, v, f, normal_face,
                                             edge_vertex_map, edge_face_map)
    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_addPoint(mfig, P, radius=0.1, color=(1, 0, 0))
    
    # different color for each face
    for f_ind in F:
        face_verts = v[f[f_ind,:],:]
        graphics.mayavi_addMesh(mfig, face_verts,[(0, 1, 2)], color=tuple(np.random.rand(3)))
    
    # draw the points which make up the edges and draw a line for the edge
    for v_ind in V:
        graphics.mayavi_addPoint(mfig, v[v_ind,:], radius=0.1, color=(0, 0, 1))

    graphics.mayavi_addLine(mfig, v[V[0],:], v[V[1],:], color=(0, 0, 1))
    
    graphics.mayavi_addTitle(mfig, 'Closest Edge', color=(0, 0, 0), size=0.5)


def test_closest_face_plot_cube():
    """Needs to be aligned with a face
    
        i.e. not in the corners
    """

    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')
    pt = np.array([1, 0.1, 0.1])
    D, P, F, V = wavefront.distance_to_faces(pt, v, f, 
                                             ast.asteroid_grav['normal_face'])
    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)
    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_addTitle(mfig, 'Closest Face', color=(0, 0, 0), size=0.5)

    if D:
        graphics.mayavi_addPoint(mfig, P, radius=0.1, color=(1, 0, 0))
        # different color for each face
        graphics.mayavi_addMesh(mfig, V,[(0, 1, 2)], color=tuple(np.random.rand(3)))

def test_closest_face_plot_asteroid():
    """Needs to be aligned with a face
    
        i.e. not in the corners
    """

    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = ast.asteroid_grav['V'], ast.asteroid_grav['F']

    pt = np.array([1, 0, 0])
    D, P, F, V = wavefront.distance_to_faces(pt, v, f, 
                                             ast.asteroid_grav['normal_face'])
    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)
    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    graphics.mayavi_addTitle(mfig, 'Closest Face', color=(0, 0, 0), size=0.5)

    if D:
        graphics.mayavi_addPoint(mfig, P, radius=0.1, color=(1, 0, 0))
        # different color for each face
        graphics.mayavi_addMesh(mfig, V,[(0, 1, 2)], color=tuple(np.random.rand(3)))

if __name__ == "__main__":
    test_normal_face_plot()

    test_closest_vertex_plot_cube()
    test_closest_vertex_plot_asteroid()

    test_closest_edge_plot_cube()
    test_closest_edge_plot_asteroid()

    test_closest_face_plot_cube()
    test_closest_face_plot_asteroid()

