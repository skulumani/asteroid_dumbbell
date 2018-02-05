"""Script/Functions to generate plots for the mesh update algorithm
"""
from point_cloud import wavefront
from visualization import graphics

def cube_mesh_with_vertices_edges_faces():
    """Plot the example cube with all faces, vertices, and edges
    """

    # read the cube
    v, f = wavefront.read_obj('./integration/cube.obj')

    # create the figure
    mfig = graphics.mayavi_figure(size=(1024, 1068),
                                  bg=(1, 1, 1))

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

    # save the figure to eps

     
if __name__ == "__main__":
    cube_mesh_with_vertices_edges_faces()
