from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
from point_cloud import wavefront
from dynamics import asteroid
import pdb
from mayavi import mlab
import mayavi.api

def define_grid(ast, density):
    # generate a grid around the asteroid
    nx, ny, nz = (5, 5, 5)
    xmin, xmax = -ast.axes[0], ast.axes[0]
    ymin, ymax = -ast.axes[1], ast.axes[1]
    zmin, zmax = -ast.axes[2], ast.axes[2]

    xg, yg, zg = np.mgrid[ xmin:xmax:5j, ymin:ymax:5j, zmin:zmax:5j]
    
    grid = {'xg': xg, 'yg':yg, 'zg':zg,
            'nx': nx, 'ny':ny, 'nz':nz}
    return grid

def generate_scalar_data(ast, grid):
    nx = grid['nx']
    ny = grid['ny']
    nz = grid['nz']
    
    xg = grid['xg']
    yg = grid['yg']
    zg = grid['zg']

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

def visualize_data(ast, Ug_array, grid):

    nx = grid['nx']
    ny = grid['ny']
    nz = grid['nz']
    
    xg = grid['xg']
    yg = grid['yg']
    zg = grid['zg']
    # plot some contour plots in mayavi
    # engine = mayavi.api.Engine()
    # engine.start()
    # scene = engine.scenes[0]

    potential_fig = mlab.figure(figure=None)
    mesh = wavefront.draw_polyhedron_mayavi(ast.V, ast.F, potential_fig)                
    # contour plot
    # contour3d = mlab.contour3d(xg, yg, zg, Ug_array, figure=potential_fig)
    # volume rendering
    # volume = mlab.pipeline.volume(mlab.pipeline.scalar_field(Ug_array))
    f = mlab.pipeline.scalar_field(xg, yg, zg, Ug_array)
    v = mlab.pipeline.volume(f, vmin=0, vmax=0.2)
    
    # cut planes
    mlab.pipeline.image_plane_widget(f, 
                                     plane_orientation='x_axes',
                                     slice_index=nx/2)
    mlab.pipeline.image_plane_widget(f, 
                                     plane_orientation='y_axes',
                                     slice_index=ny/2)
    mlab.pipeline.image_plane_widget(f, 
                                     plane_orientation='z_axes',
                                     slice_index=nz/2)
    mlab.outline()
    mlab.axes()
    mlab.show()

def comparison_generate_asteroid():
    """Generate all of the asteroid objects
    """
    
    # define all of the asteroid models
    name = 'itokawa'
    ast_mat_4092 = asteroid.Asteroid(name, 4092, 'mat')
    ast_mat_2048 = asteroid.Asteroid(name, 2048, 'mat')
    # OBJ - original without reduction
    ast_obj_low = asteroid.Asteroid(name, 0, 'obj')
    
    # now decimate
    ast_obj_90 = ast_obj_low
    ast_obj_90.V, ast_obj_90.F = wavefront.decimate_numpy(ast_obj_low.V, ast_obj_low.F, 0.9)
    ast_obj_90.polyhedron_shape_input()
    ast_obj_50 = ast_obj_low
    ast_obj_50.V, ast_obj_50.F = wavefront.decimate_numpy(ast_obj_low.V, ast_obj_low.F, 0.9)
    ast_obj_50.polyhedron_shape_input()
    # OBJ reconstructed - take the previous OBJ version and use surface reconstruction on them

    

    
if __name__ == '__main__':
    ast = asteroid.Asteroid('itokawa', 0, 'obj')
    print('Finished with gravity model')
    grid = define_grid(ast, 5)
    Ug_array = generate_scalar_data(ast, grid)
    visualize_data(ast, Ug_array, grid)
