# Reconstruction example and testing

from visualization import graphics
from point_cloud import wavefront

from kinematics import attitude
import numpy as np
import scipy.io

import pdb

# load data from sphere and ellipsoid
sphere_data = scipy.io.loadmat('./data/sphere_distmesh.mat')
ellipsoid_data = scipy.io.loadmat('./data/ellipsoid_distmesh.mat')

vs, fs = sphere_data['v'], sphere_data['f']-1
ve, fe = ellipsoid_data['v'], ellipsoid_data['f']-1
# vs, f = wavefront.ellipsoid_mesh(0.5, 0.5, 0.5, density=20, subdivisions=1)
# ve, fe = wavefront.ellipsoid_mesh(1, 1, 1, density=10, subdivisions=1)
# convert all vertices to spherical coordinates
v_spherical = wavefront.cartesian2spherical(vs);
# ve_spherical = wavefront.cartesian2spherical(ve);

mfig = graphics.mayavi_figure()
# vc, f = wavefront.read_obj('./integration/cube.obj')
# v_spherical = wavefront.cartesian2spherical(vc)

# mesh_param = wavefront.polyhedron_parameters(v, f)
pt = ve[0, :]
pt_spherical = wavefront.cartesian2spherical(pt)

nv_spherical, nf = wavefront.spherical_incremental_mesh_update(mfig, pt_spherical, v_spherical, fs,
                                                               surf_area=0.5, 
                                                               factor=1, 
                                                               radius_factor=0.3)

# convert back to cartesian
nv = wavefront.spherical2cartesian(nv_spherical)

# view the mesh
graphics.mayavi_addMesh(mfig, nv, nf, representation='surface')
graphics.mayavi_addPoint(mfig, pt)
