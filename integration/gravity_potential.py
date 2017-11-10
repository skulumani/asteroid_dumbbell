from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
from point_cloud import wavefront
from dynamics import asteroid
import pdb
from mayavi import mlab
import mayavi.api
import copy
from multiprocessing import Pool
from functools import partial
def define_grid(ast, density=100):
    # generate a grid around the asteroid
    nx, ny, nz = (density, density, density)
    xmin, xmax = -ast.axes[0], ast.axes[0]
    ymin, ymax = -ast.axes[1], ast.axes[1]
    zmin, zmax = -ast.axes[2], ast.axes[2]

    xg, yg, zg = np.mgrid[ xmin:xmax:100j, ymin:ymax:100j, zmin:zmax:100j]
    
    grid = {'xg': xg, 'yg':yg, 'zg':zg,
            'nx': nx, 'ny':ny, 'nz':nz}
    return grid

def generate_scalar_data(grid, ast_in):
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
                _, Ugrad, _,  _= ast_in.polyhedron_potential(pos)
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
    mlab.colorbar(object=mesh, title='mm/sec^2' )
    # mlab.show()

def iterable_visualize(ast_list, Ug_list, titles_list, grid):

    for ii in range(len(ast_list)):
        visualize_data(ast_list[ii], Ug_list[ii], grid)
        mlab.title(titles_list[ii])

if __name__ == '__main__':
    # define all of the asteroid models
    name = 'itokawa'
    ast_mat_4092 = asteroid.Asteroid(name, 4092, 'mat')
    ast_mat_2048 = asteroid.Asteroid(name, 2048, 'mat')
    # OBJ - original without reduction
    ast_obj_low = asteroid.Asteroid(name, 0, 'obj')

    # now decimate
    ast_obj_90 = copy.deepcopy(ast_obj_low)
    ast_obj_90.V, ast_obj_90.F = wavefront.decimate_numpy(ast_obj_low.V, ast_obj_low.F, 0.9)
    ast_obj_90.asteroid_grav = ast_obj_90.polyhedron_shape_input()
    ast_obj_50 = copy.deepcopy(ast_obj_low)
    ast_obj_50.V, ast_obj_50.F = wavefront.decimate_numpy(ast_obj_low.V, ast_obj_low.F, 0.9)
    ast_obj_50.asteroid_grav = ast_obj_50.polyhedron_shape_input()

    # OBJ reconstructed - take the previous OBJ version and use surface reconstruction on them
    ast_obj_90_reconstruct = copy.deepcopy(ast_obj_90)
    ast_obj_90_reconstruct.V, ast_obj_90_reconstruct.F = wavefront.reconstruct_numpy(ast_obj_90.V)
    ast_obj_90_reconstruct.asteroid_grav = ast_obj_90_reconstruct.polyhedron_shape_input()

    ast_obj_50_reconstruct = ast_obj_50
    ast_obj_50_reconstruct.V, ast_obj_50_reconstruct.F = wavefront.reconstruct_numpy(ast_obj_50.V)
    ast_obj_50_reconstruct.asteroid_grav = ast_obj_50_reconstruct.polyhedron_shape_input()

    ast_obj_low_reconstruct = copy.deepcopy(ast_obj_low)
    ast_obj_low_reconstruct.V, ast_obj_low_reconstruct.F = wavefront.reconstruct_numpy(ast_obj_low.V)
    ast_obj_low_reconstruct.asteroid_grav = ast_obj_low_reconstruct.polyhedron_shape_input()

    ast_list = ( ast_mat_4092, ast_mat_2048, ast_obj_low,
                ast_obj_90, ast_obj_50, 
                ast_obj_90_reconstruct, ast_obj_50_reconstruct, ast_obj_low_reconstruct )
    titles_list = ( 'MAT 4092', 'MAT 2048', 'OBJ Low',
                    'OBJ Decimate 0.90', 'OBJ Decimate 0.50',
                    'OBJ Reconstruct 0.90', 'OBJ Reconstruct 0.50', 'OBJ Low Reconstruct' )
    grid = define_grid(ast_obj_low)
    with Pool(8) as p:
        func = partial(generate_scalar_data, grid)
        Ug_list = p.map(func, ast_list)

        # visualize_data(ast, Ug, grid)
        # mlab.title(title)
    np.savez('./integration/gravity_potential.npz',ast_list=ast_list,
                grid=grid, Ug_list=Ug_list, titles_list=titles_list)
