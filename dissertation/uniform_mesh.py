"""Uniform mesh plots for dissertation

Some examples of variations in a initial mesh estimate for the paper

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""

import numpy as np
from lib import surface_mesh
from visualization import graphics

# generate two triaxial ellipsoids
smesh_1 = surface_mesh.SurfMesh(1, 0.5, 0.5, 10, 0.2, 0.1)
smesh_2 = surface_mesh.SurfMesh(1, 0.5, 0.5, 10, 0.1, 0.1)

view =(45.00000000000001,
       54.73561031724535,
       2.705956013687095,
       np.array([ 0.12708219, -0.05595058, -0.07152687]))
 
mfig = graphics.mayavi_figure(offscreen=False)
mesh = graphics.mayavi_addMesh(mfig, smesh_1.get_verts(), smesh_1.get_faces(), representation='wireframe')
graphics.mlab.view(*view)
graphics.mlab.savefig('/tmp/uniform_mesh_coarse.jpg', magnification=4)

ms = mesh.mlab_source
ms.reset(x=smesh_2.get_verts()[:, 0], y=smesh_2.get_verts()[:, 1], z=smesh_2.get_verts()[:, 2],
         triangles=smesh_2.get_faces())
graphics.mlab.view(*view)
graphics.mlab.savefig('/tmp/uniform_mesh_fine.jpg', magnification=4)

