"""Module to test out some wavefront function with plots
"""
import numpy as np

from point_cloud import wavefront
from visualization import graphics

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

if __name__ == "__main__":
    test_normal_face_plot()


