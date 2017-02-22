import dynamics.asteroid as asteroid
import attitude_ref.attitude as attitude

import numpy as np
import scipy as sp

import matplotlib as mpl
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import matplotlib.colors as colors

def vertex_plotter(ast, fig):

    ax = axes3d.Axes3D(fig)

    for ii in np.arange(len(ast.F)):
        facet = [ast.asteroid_grav.get('V1')[ii,:], ast.asteroid_grav.get('V2')[ii,:], ast.asteroid_grav.get('V3')[ii,:]]
        tri = axes3d.art3d.Poly3DCollection([facet])
        tri.set_color('g')    
        tri.set_edgecolor('k')
        tri.set_alpha(1.0)
        ax.add_collection3d(tri)

    return 0

def plot_trajectory(pos, fig):
    """Plot the state trajectory in a 3D view

    """

    traj_ax = fig.gca(projection='3d')
    traj_ax.set_xlim([-3, 3])
    traj_ax.set_ylim([-3, 3])
    traj_ax.set_zlim([-3, 3])
    # plotting.vertex_plotter(ast, traj_fig)

    traj_ax.plot(pos[:,0],pos[:,1],pos[:,2])

    return 0
    
if __name__ == '__main__':
    ast = asteroid.Asteroid('itokawa',32)
    vertex_plotter(ast,plt.figure())

    plt.show()
