# Reconstruction example and testing

from visualization import graphics
from point_cloud import wavefront
import numpy as np

vs, fs = wavefront.ellipsoid_mesh(0.5, 0.5, 0.5, density=10, subdivisions=1)
ve, fe = wavefront. ellipsoid_mesh(1, 1, 1, density=10, subdivisions=1)

# convert all vertices to spherical coordinates

mesh_param = wavefront.polyhedron_parameters(v, f)

pt = np.array([1, 1, 1])

nv, nf = wavefront.radius_mesh_incremental_update(pt, v, f, mesh_param,
                                                  max_angle=np.deg2rad(10), angle_std=3)


# view the mesh
mfig = graphics.mayavi_figure()
graphics.mayavi_addMesh(mfig, nv, nf)
graphics.mayavi_addPoint(mfig, pt)
