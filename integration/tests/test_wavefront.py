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

def plot_data(pt, v, f, D, P, V, E, F, string='Closest Primitive'):

    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=0.1, color=(0, 1, 0))
    if P.any():
        graphics.mayavi_points3d(mfig, P, scale_factor=0.1, color=(1, 0, 0))

    # different color for each face
    if F.size:
        try:
            _ = iter(F[0])
            for f_list in F:
                for f_ind in f_list:
                    face_verts = v[f[f_ind,:],:]
                    graphics.mayavi_addMesh(mfig, face_verts, [(0, 1, 2)], color=tuple(np.random.rand(3)))
        except IndexError as err:
            face_verts = v[f[F,:],:]
            graphics.mayavi_addMesh(mfig, face_verts, [(0, 1, 2)], color=tuple(np.random.rand(3)))
        except (TypeError,) as err:
            for f_ind in F:
                face_verts = v[f[f_ind,:],:]
                graphics.mayavi_addMesh(mfig, face_verts, [(0, 1, 2)], color=tuple(np.random.rand(3)))

    
    # draw the points which make up the edges and draw a line for the edge
    if V.size:
        try:
            _ = iter(V)
            for v_ind in V:
                graphics.mayavi_addPoint(mfig, v[v_ind,:], radius=0.1, color=(0, 0, 1))
        except TypeError as err:
            graphics.mayavi_addPoint(mfig, v[V, :], radius=0.1, color=(0, 0, 1))
    
    # draw edges
    if E.size:
        try:
            _ = iter(E[0][0])
            for e_list in E:
                for e_ind in e_list:
                    graphics.mayavi_addLine(mfig, v[e_ind[0],:], v[e_ind[1], :], color=(0, 0, 0))
        except IndexError as err:
            graphics.mayavi_addLine(mfig, v[E[0],:], v[E[1], :], color=(0, 0, 0))
        except (TypeError,) as err:

            for e_ind in E:
                graphics.mayavi_addLine(mfig, v[e_ind[0],:], v[e_ind[1], :], color=(0, 0, 0))

    graphics.mayavi_addTitle(mfig, string, color=(0, 0, 0), size=0.5)

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

def test_closest_vertex_plot_cube(pt=np.random.uniform(0.6 * np.sqrt(3), 1) * sphere.rand(2)):

    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')

    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_vertices(pt, v, f, 
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)

    plot_data(pt, v, f, D, P, V, E, F, 'Closest Vertex')

    return D, P, V, E, F

def test_closest_vertex_plot_asteroid(pt=np.random.uniform(1, 1.5) * sphere.rand(2)):

    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v,f = ast.asteroid_grav['V'], ast.asteroid_grav['F']
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_vertices(pt, v, f, 
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)

    plot_data(pt, v, f, D, P, V, E, F, 'Closest Vertex')
    return D, P, V, E, F

def test_closest_edge_plot_cube(pt=np.random.uniform(0.9, 1.5)*sphere.rand(2)):
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')
    
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_edges(pt, v, f, normal_face,
                                             edge_vertex_map, edge_face_map,
                                             vf_map)
    plot_data(pt, v, f, D, P, V, E, F, 'Closest Edge')
    return D, P, V, E, F

def test_closest_edge_plot_asteroid(pt=np.random.uniform(0.8, 1.5)*sphere.rand(2)):
    ast = asteroid.Asteroid('castalia', 256, 'mat')
    
    v,f = ast.asteroid_grav['V'], ast.asteroid_grav['F']
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_edges(pt, v, f, normal_face,
                                             edge_vertex_map, edge_face_map,
                                             vf_map)
    plot_data(pt, v, f, D, P, V, E, F, 'Closest Edge')
    return D, P, V, E, F


def test_closest_face_plot_cube(pt=np.random.uniform(0.8, 1.5)*sphere.rand(2)):
    """Needs to be aligned with a face
    
        i.e. not in the corners
    """

    ast = asteroid.Asteroid('castalia', 256, 'mat')
    v, f = wavefront.read_obj('./integration/cube.obj')
    ast = ast.loadmesh(v, f, 'cube')
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_faces(pt, v, f, 
                                                normal_face, 
                                                edge_vertex_map,
                                                edge_face_map,
                                                vf_map)
    plot_data(pt, v, f, D, P, V, E, F, 'Closest Face')

    return D, P, V, E, F

def test_closest_face_plot_asteroid(pt=np.random.uniform(0.8, 1.5)*sphere.rand(2)):
    """Needs to be aligned with a face
    
        i.e. not in the corners
    """

    ast = asteroid.Asteroid('castalia', 256, 'mat')

    v,f = ast.asteroid_grav['V'], ast.asteroid_grav['F']
    edge_vertex_map = ast.asteroid_grav['edge_vertex_map']
    edge_face_map = ast.asteroid_grav['edge_face_map']
    normal_face = ast.asteroid_grav['normal_face']
    vf_map = ast.asteroid_grav['vertex_face_map']

    D, P, V, E, F = wavefront.distance_to_faces(pt, v, f, 
                                                normal_face, 
                                                edge_vertex_map,
                                                edge_face_map,
                                                vf_map)
    plot_data(pt, v, f, D, P, V, E, F, 'Closest Face')

    return D, P, V, E, F

def test_distance_to_mesh(pt=np.random.uniform(0.8, 1.5)*sphere.rand(2)):
    """Test out the point processing function
    """
    v, f = wavefront.read_obj('./integration/cube.obj')
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    D, P, V, E, F = wavefront.distance_to_mesh(pt, v, f, mesh_parameters)

    plot_data(pt, v, f, D, P, V, E, F, 'Minimum distance to mesh')
    
    return D, P, V, E, F

if __name__ == "__main__":
    test_normal_face_plot()

    test_closest_vertex_plot_cube()
    test_closest_vertex_plot_asteroid()

    test_closest_edge_plot_cube()
    test_closest_edge_plot_asteroid()

    test_closest_face_plot_cube()
    test_closest_face_plot_asteroid()

