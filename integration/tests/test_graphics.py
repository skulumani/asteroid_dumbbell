"""Module to run some visual tests by creating plots and having 
the user visually compare things to look good
"""

from point_cloud import wavefront
from visualization import graphics
from dynamics import asteroid
from mayavi import mlab
import pdb
import threading

def mlab_show():
    mlab.show()
    print('Opened MLAB figures')
    return

def test_reconstruct():
    """Read a OBJ file, reconstruct it's surface using VTK and plot
    """
    filename = './data/shape_model/ITOKAWA/itokawa_low.obj'
    ratio = 0.5
    v, f = wavefront.read_obj(filename)
    dv, df = wavefront.decimate_numpy(v, f, ratio)
    rv, rf = wavefront.reconstruct_numpy(v)

    # create some figures
    orig_fig = mlab.figure()
    reconstruct_fig = mlab.figure()
    _ = graphics.draw_polyhedron_mayavi(v, f, orig_fig)
    mlab.title('Original OBJ')
    _ = graphics.draw_polyhedron_mayavi(rv, rf, reconstruct_fig)
     
    mlab.title('Reconstructed OBJ')

    # display both and complete test
    print('Now compare the two images')

class TestPolyhedronPlotting():
    polydata = wavefront.read_obj_to_polydata('./data/shape_model/EROS/eros_medium.obj')

    def test_draw_vtk_polyhedron(self):
        renderer = graphics.vtk_renderer()
        
        graphics.vtk_addPoly(renderer, self.polydata)
        graphics.vtk_show(renderer)

    def test_draw_mayavi_polyhedron(self):
        fig = graphics.mayavi_figure()
        mesh = graphics.mayavi_addPoly(fig, self.polydata)

if __name__ == '__main__':
    TestPolyhedronPlotting().test_draw_vtk_polyhedron()
    TestPolyhedronPlotting().test_draw_mayavi_polyhedron()
    test_reconstruct()
