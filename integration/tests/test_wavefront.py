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

def plot_data(pt, v, f, D, P, V, E, F, string='Closest Primitive', radius=0.1):

    # draw the mayavi figure
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, v, f)

    graphics.mayavi_addPoint(mfig, pt, radius=radius, color=(0, 1, 0))
    if P.any():
        graphics.mayavi_points3d(mfig, P, scale_factor=radius, color=(1, 0, 0))

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
                graphics.mayavi_addPoint(mfig, v[v_ind,:], radius=radius, color=(0, 0, 1))
        except TypeError as err:
            graphics.mayavi_addPoint(mfig, v[V, :], radius=radius, color=(0, 0, 1))
    
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

    plot_data(pt, v, f, D, P, V, E, F, '')

    return D, P, V, E, F

def test_closest_vertex_plot_asteroid(pt=np.random.uniform(0.5, 1) * sphere.rand(2)):

    ast = asteroid.Asteroid('itokawa',0, 'obj')
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
    plot_data(pt, v, f, D, P, V, E, F, '')
    return D, P, V, E, F

def test_closest_edge_plot_asteroid(pt=np.random.uniform(0.8, 1.5)*sphere.rand(2)):
    ast = asteroid.Asteroid('itokawa', 0, 'obj')
    
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
    plot_data(pt, v, f, D, P, V, E, F, '')

    return D, P, V, E, F

def test_closest_face_plot_asteroid(pt=np.random.uniform(0.8, 1.5)*sphere.rand(2)):
    """Needs to be aligned with a face
    
        i.e. not in the corners
    """

    ast = asteroid.Asteroid('itokawa', 0, 'obj')

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
    D, P, V, E, F, primitive = wavefront.distance_to_mesh(pt, v, f, mesh_parameters)
    plot_data(pt, v, f, D, P, V, E, F, 'Min Primitive: ' + primitive)
    
    return D, P, V, E, F, primitive

def test_vertex_insertion(pt=np.array([1, 1, 1])):
    v, f = wavefront.read_obj('./integration/cube.obj')
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    D, P, V, E, F, primitive = wavefront.distance_to_mesh(pt, v, f, mesh_parameters)
    nv, nf = wavefront.vertex_insertion(pt, v ,f, D, P, V, E, F)
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, nv, nf)
    
    return D, P, V, E, F, primitive

def test_edge_insertion(pt=np.array([1, 1, 0])):
    v, f = wavefront.read_obj('./integration/cube.obj')
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    D, P, V, E, F, primitive = wavefront.distance_to_mesh(pt, v, f, mesh_parameters)
    nv, nf = wavefront.edge_insertion(pt, v,f, D, P, V, E, F)
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, nv, nf)

    return D, P, V, E, F, primitive

def test_face_insertion(pt=np.array([1, 0.1, 0])):
    v, f = wavefront.read_obj('./integration/cube.obj')
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    D, P, V, E, F, primitive = wavefront.distance_to_mesh(pt, v, f, mesh_parameters)
    nv, nf = wavefront.face_insertion(pt, v ,f, D, P, V, E, F)
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, nv, nf)

def test_point_insertion_random():

    num_points = 100
    nv, nf = wavefront.read_obj('./integration/cube.obj')
    

    # loop over random points and add them to the cube
    for ii in range(num_points):

        pt = np.random.uniform(0.6, 0.7)*sphere.rand(2)
        mesh_parameters = wavefront.polyhedron_parameters(nv, nf)
        D, P, V, E, F, primitive = wavefront.distance_to_mesh(pt, nv, nf, mesh_parameters)
        if primitive == 'vertex':
            nv, nf = wavefront.vertex_insertion(pt, nv, nf, D, P, V, E, F)
        elif primitive == 'edge':
            nv, nf = wavefront.edge_insertion(pt, nv, nf, D, P, V, E, F)
        elif primitive == 'face':
            nv, nf = wavefront.face_insertion(pt, nv, nf, D, P, V, E, F)

    mfig = graphics.mayavi_figure()
    mesh = graphics.mayavi_addMesh(mfig, nv, nf)

def test_radius_mesh_update_cube(pt=np.array([1, 0, 0])):
    """Update the mesh by modifying the radius of the closest point
    """
    # load the cube
    v, f = wavefront.read_obj('./integration/cube.obj')
    # pick a point
    nv, nf = wavefront.radius_mesh_incremental_update(pt, v, f)

    # plot the new mesh
    mfig = graphics.mayavi_figure()
    graphics.mayavi_addMesh(mfig, nv, nf)
    graphics.mayavi_addPoint(mfig, pt)
    graphics.mayavi_points3d(mfig, v, color=(0,  1, 0))

def test_radius_cube_into_sphere():
    """Transform a cube into a sphere
    """
    vc, fc = wavefront.read_obj('./integration/cube.obj')
    vs, fs = wavefront.ellipsoid_mesh(2, 2, 2, density=10, subdivisions=0)

    mfig = graphics.mayavi_figure()
    mesh = graphics.mayavi_addMesh(mfig, vc,fc)
    graphics.mayavi_points3d(mfig, vc, color=(0, 1, 0))
    ms = mesh.mlab_source
    for pt in vs:
        vc, fc = wavefront.radius_mesh_incremental_update(pt, vc, fc)
        ms.reset(x=vc[:, 0], y=vc[:, 1], z=vc[:, 2], triangles=fc)
        graphics.mayavi_addPoint(mfig, pt)

def test_radius_sphere_into_ellipse():
    """See if we can turn a sphere into an ellipse by changing the radius of
    vertices

    """
    # TODO Try with different densities

    # define the sphere
    vs, fs = wavefront.ellipsoid_mesh(1, 1, 1, density=20, subdivisions=0)
    ve, fe = wavefront.ellipsoid_mesh(2, 3, 4, density=30, subdivisions=2)
    
    mfig = graphics.mayavi_figure()
    mesh = graphics.mayavi_addMesh(mfig, vs, fs)
    ms = mesh.mlab_source
    # in a loop add each vertex of the ellipse into the sphere mesh
    for pt in ve:
        vs, fs = wavefront.radius_mesh_incremental_update(pt, vs,fs)
        ms.reset(x=vs[:,0], y=vs[:,1], z=vs[:,2], triangles=fs)

    # visualize the mesh


if __name__ == "__main__":
    test_normal_face_plot()

    test_closest_vertex_plot_cube()
    test_closest_vertex_plot_asteroid()

    test_closest_edge_plot_cube()
    test_closest_edge_plot_asteroid()

    test_closest_face_plot_cube()
    test_closest_face_plot_asteroid()

