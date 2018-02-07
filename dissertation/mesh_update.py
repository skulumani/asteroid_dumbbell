"""Generate example cube plots for dissertation

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
import os

import numpy as np

from point_cloud import wavefront
from visualization import graphics

view = {'azimuth': 16.944197132093564, 'elevation': 66.34177792039738,
        'distance': 2.9356815748114435, 
        'focalpoint': np.array([0.20105769, -0.00420018, -0.016934045])}

def plot_data(pt, v, f, D, P, V, E, F, string='Closest Primitive', radius=0.1,
              size=(800*1.6, 800)):

    # draw the mayavi figure
    mfig = graphics.mayavi_figure(size=size)
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
    return mfig

def cube_mesh_with_vertices_edges_faces(img_path):
    """Plot the example cube with all faces, vertices, and edges
    """
    filename = os.path.join(img_path,'cube_mesh.jpg')

    size = (800*1.618, 800) 
    # read the cube
    v, f = wavefront.read_obj('./integration/cube.obj')

    # create the figure
    mfig = graphics.mayavi_figure(size=size, bg=(1, 1, 1))

    # draw the mesh
    mesh = graphics.mayavi_addMesh(mfig, v, f, 
                                   color=(0.5, 0.5, 0.5),
                                   representation='surface')

    # draw all the vertices
    points = graphics.mayavi_points3d(mfig, v, scale_factor=0.1, 
                                      color=(0, 0, 1)) 

    # draw the edges
    mesh_edges = graphics.mayavi_addMesh(mfig, v, f, 
                                         color=(1, 0, 0),
                                         representation='mesh')
    graphics.mlab.view(azimuth=view['azimuth'], elevation=view['elevation'],
                       distance=view['distance'], focalpoint=view['focalpoint'],
                       figure=mfig)

    # save the figure to eps
    graphics.mlab.savefig(filename, magnification=4)

def cube_closest_vertex(img_path):
    filename = os.path.join(img_path, 'cube_closest_vertex.jpg')

    size = (800*1.618, 800) 
    pt = np.array([0.7, 0.7, 0.7])
    v, f = wavefront.read_obj('./integration/cube.obj')
    
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    edge_vertex_map = mesh_parameters.edge_vertex_map
    edge_face_map = mesh_parameters.edge_face_map
    normal_face = mesh_parameters.normal_face
    vf_map = mesh_parameters.vertex_face_map

    D, P, V, E, F = wavefront.distance_to_vertices(pt, v, f, 
                                                   normal_face,
                                                   edge_vertex_map,
                                                   edge_face_map,
                                                   vf_map)
    
    mfig = plot_data(pt, v, f, D, P, V, E, F, '')
    graphics.mlab.view(azimuth=view['azimuth'], elevation=view['elevation'],
                       distance=view['distance'], focalpoint=view['focalpoint'],
                       figure=mfig)

    graphics.mlab.savefig(filename, magnification=4)

    return mfig

def cube_closest_edge(img_path):
    filename = os.path.join(img_path, 'cube_closest_edge.jpg')

    size = (800*1.618, 800) 
    pt = np.array([0.7, 0.7, 0])
    v, f = wavefront.read_obj('./integration/cube.obj')
    
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    edge_vertex_map = mesh_parameters.edge_vertex_map
    edge_face_map = mesh_parameters.edge_face_map
    normal_face = mesh_parameters.normal_face
    vf_map = mesh_parameters.vertex_face_map

    D, P, V, E, F = wavefront.distance_to_edges(pt, v, f, 
                                                normal_face,
                                                edge_vertex_map,
                                                edge_face_map,
                                                vf_map)
    
    mfig = plot_data(pt, v, f, D, P, V, E, F, '')
    graphics.mlab.view(azimuth=view['azimuth'], elevation=view['elevation'],
                       distance=view['distance'], focalpoint=view['focalpoint'],
                       figure=mfig)

    graphics.mlab.savefig(filename, magnification=4)

    return mfig

def cube_closest_face(img_path):
    filename = os.path.join(img_path, 'cube_closest_face.jpg')

    size = (800*1.618, 800) 
    pt = np.array([0.7, 0.2, 0])
    v, f = wavefront.read_obj('./integration/cube.obj')
    
    mesh_parameters = wavefront.polyhedron_parameters(v, f)
    edge_vertex_map = mesh_parameters.edge_vertex_map
    edge_face_map = mesh_parameters.edge_face_map
    normal_face = mesh_parameters.normal_face
    vf_map = mesh_parameters.vertex_face_map

    D, P, V, E, F = wavefront.distance_to_faces(pt, v, f, 
                                                normal_face,
                                                edge_vertex_map,
                                                edge_face_map,
                                                vf_map)
    
    mfig = plot_data(pt, v, f, D, P, V, E, F, '')
    graphics.mlab.view(azimuth=view['azimuth'], elevation=view['elevation'],
                       distance=view['distance'], focalpoint=view['focalpoint'],
                       figure=mfig)

    graphics.mlab.savefig(filename, magnification=4)

    return mfig
if __name__ == "__main__":
    img_path = '/tmp/mayavi_figure'
    if not os.path.exists(img_path):
        os.makedirs(img_path)

    cube_mesh_with_vertices_edges_faces(img_path)
    cube_closest_vertex(img_path)
    cube_closest_edge(img_path)
    cube_closest_face(img_path)
