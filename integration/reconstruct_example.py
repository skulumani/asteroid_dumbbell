# Reconstruction example and testing

from visualization import graphics
from point_cloud import wavefront
import numpy as np

vs, fs = wavefront.ellipsoid_mesh(0.5, 0.5, 0.5, density=20, subdivisions=1)
ve, fe = wavefront.ellipsoid_mesh(1, 1, 1, density=20, subdivisions=1)

# convert all vertices to spherical coordinates
vs_spherical = wavefront.cartesian2spherical(vs);
ve_spherical = wavefront.cartesian2spherical(ve);

# mesh_param = wavefront.polyhedron_parameters(v, f)

pt = np.array([1, 1, 1])

pt_spherical = wavefront.cartesian2spherical(pt)

nv_spherical, nf = wavefront.spherical_incremental_mesh_update(pt_spherical, vs_spherical, fs, surf_area=0.1, factor=1)

# convert back to cartesian
nv = wavefront.spherical2cartesian(nv_spherical)
# view the mesh
mfig = graphics.mayavi_figure()
graphics.mayavi_addMesh(mfig, nv, nf)
graphics.mayavi_addPoint(mfig, pt)
