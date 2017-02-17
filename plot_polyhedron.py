import dynamics.asteroid as asteroid
import attitude_ref.attitude as attitude

import numpy as np
import scipy as sp

import pdb

import matplotlib as mpl
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import matplotlib.colors as colors

ast = asteroid.Asteroid('castalia',4096)

ax = axes3d.Axes3D(plt.figure())
ax.set_xlim([-ast.axes[0], ast.axes[0]])
ax.set_ylim([-ast.axes[1], ast.axes[1]])
ax.set_zlim([-ast.axes[2], ast.axes[2]])

for ii in np.arange(len(ast.F)):
    facet = [ast.asteroid_grav.get('V1')[ii,:], ast.asteroid_grav.get('V2')[ii,:], ast.asteroid_grav.get('V3')[ii,:]]
    tri = axes3d.art3d.Poly3DCollection([facet])
    tri.set_color('g')    
    tri.set_edgecolor('k')
    tri.set_alpha(1.0)
    ax.add_collection3d(tri)

plt.show()
