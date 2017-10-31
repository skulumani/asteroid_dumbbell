from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
from point_cloud import wavefront
from dynamics import asteroid
import pdb
from mayavi import mlab
import mayavi.api

def define_grid(ast):
    # generate a grid around the asteroid
    nx, ny, nz = (5,5,5)
    xmin, xmax = -ast.axes[0], ast.axes[0]
    ymin, ymax = -ast.axes[1], ast.axes[1]
    zmin, zmax = -ast.axes[2], ast.axes[2]

    xg, yg, zg = np.mgrid[ xmin:xmax:5j, ymin:ymax:5j, zmin:zmax:5j]
    
    return 
def generate_scalar_data(ast):
    Ug_array = np.zeros((nx, ny, nz))
    # fidn the potential at each point
    for ii in range(nx):
        for jj in range(ny):
            for kk in range(nz):
                pos = np.array([xg[ii, jj, kk], yg[ii, jj, kk], zg[ii, jj, kk]])
                _, Ugrad, _,  _= ast.polyhedron_potential(pos)
                Ug_array[ii, jj, kk] = np.linalg.norm(Ugrad)
    # scale potential to useful units - convert from km to mm
    km2mm = 1e7
    Ug_array = Ug_array*km2mm # now in mm / sec^2

    return Ug_array

def visualize_data(ast, Ug_array):

    # plot some contour plots in mayavi
    engine = mayavi.api.Engine()
    engine.start()

    potential_fig = mlab.figure(figure=None, engine=engine)
    mesh = wavefront.draw_polyhedron_mayavi(ast.V, ast.F, potential_fig)                
    # contour plot
    # contour3d = mlab.contour3d(xg, yg, zg, Ug_array, figure=potential_fig)
    # volume rendering
    mlab.pipeline.volume(mlab.pipeline.scalar_field(xg, yg, zg,Ug_array))
    scene = engine.scenes[0]

    mlab.show()

if __name__ == '__main__':
    ast = asteroid.Asteroid('itokawa', 128, 'obj')
    Ug_array = generate_scalar_data()
    visualize_data(Ug_array)
# handle for plot
# iso_surface = scene.children[0].children[0].children[0]
# iso_surface.contour.print_traits()

# iso_surface.compute_normals=False
# iso_surface.contour.number_of_contours = 10
# iso_surface.contour.minimum_contour = 0
# iso_surface.contour.maximum_contour = 0.2
# scalar_cut_plane = ScalarCutPlane()
# module_manager1 = engine.scenes[0].children[1].children[0]
# engine.add_filter(scalar_cut_plane, module_manager1)
# scalar_cut_plane.warp_scalar.filter.normal = array([ 1.,  0.,  0.])
# scalar_cut_plane.implicit_plane.widget.normal = array([ 1.,  0.,  0.])
# scalar_cut_plane.implicit_plane.widget.origin = array([ 0.,  0.,  0.])
# scalar_cut_plane.warp_scalar.filter.normal = array([ 1.,  0.,  0.])
# scalar_cut_plane.implicit_plane.widget.normal = array([ 1.,  0.,  0.])
# scalar_cut_plane.implicit_plane.widget.origin = array([ 0.,  0.,  0.])
# scalar_cut_plane.implicit_plane.widget.normal_to_x_axis = True
