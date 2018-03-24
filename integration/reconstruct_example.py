# Reconstruction example and testing

from visualization import graphics
from point_cloud import wavefront

from kinematics import attitude
import numpy as np
import pdb
vs, f = wavefront.ellipsoid_mesh(0.5, 0.5, 0.5, density=20, subdivisions=1)
# ve, fe = wavefront.ellipsoid_mesh(1, 1, 1, density=10, subdivisions=1)
# convert all vertices to spherical coordinates
v_spherical = wavefront.cartesian2spherical(vs);
# ve_spherical = wavefront.cartesian2spherical(ve);

# vc, f = wavefront.read_obj('./integration/cube.obj')
# v_spherical = wavefront.cartesian2spherical(vc)

# mesh_param = wavefront.polyhedron_parameters(v, f)
pt = np.array([-1, 0.1, 0])
pt_spherical = wavefront.cartesian2spherical(pt)

nv_spherical, nf = wavefront.spherical_incremental_mesh_update(pt_spherical, v_spherical, f,
                                                            surf_area=0.5, 
                                                            factor=1, 
                                                            radius_factor=0.5)

# convert back to cartesian
nv = wavefront.spherical2cartesian(nv_spherical)

# view the mesh
mfig = graphics.mayavi_figure()
graphics.mayavi_addMesh(mfig, nv, nf, representation='surface')
graphics.mayavi_addPoint(mfig, pt)
