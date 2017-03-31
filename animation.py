# Visualize using Mayavi
from mayavi import mlab
import numpy as np 
import dynamics.asteroid as asteroid

# plot the asteroid as a triangular mesh

ast = asteroid.Asteroid('castalia', 64)

V = ast.asteroid_grav.get('V')
F = ast.asteroid_grav.get('F')
F = F.astype(int)
# surface = mlab.pipeline.triangular_mesh_source(V[:,0], V[:,1], V[:,2], F[0:33,:])
# numpy array of points which define the vertices of the surface
points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0]])
# numpy array defining the triangle which connects those points
element = np.array([[0, 1, 2], [0, 1, 2]])

# mlab surface defined with the points and element
surface = mlab.triangular_mesh(points[:, 0], points[:, 1], points[:, 2], element)

mlab.show()
# mlab.triangular_mesh(V[:,0], V[:,1], V[:,2], F)

# mlab.show()