# Reconstruction example and testing

from visualization import graphics
from point_cloud import wavefront
from lib import surface_mesh

from kinematics import attitude
import numpy as np
import scipy.io

import pdb

# load data from sphere and ellipsoid
# sphere_data = scipy.io.loadmat('./data/sphere_distmesh.mat')
# ellipsoid_data = scipy.io.loadmat('./data/ellipsoid_distmesh.mat')
# vs, fs = sphere_data['v'], sphere_data['f']
# ve, fe = ellipsoid_data['v'], ellipsoid_data['f']

# vs, f = wavefront.ellipsoid_mesh(0.5, 0.5, 0.5, density=20, subdivisions=1)
# ve, fe = wavefront.ellipsoid_mesh(1, 1, 1, density=10, subdivisions=1)
sphere= surface_mesh.SurfMesh(0.5, 0.5, 0.5, 10, 0.05, 0.5)
vs, fs = sphere.verts(), sphere.faces()

max_angle = wavefront.spherical_surface_area(0.5, surf_area=0.05)

mfig = graphics.mayavi_figure()

# mesh_param = wavefront.polyhedron_parameters(v, f)
pt = np.array([1, 0, 0])

vert_weight = np.full(vs.shape[0], (np.pi * 0.5 )**2)

vs, vert_weight = wavefront.spherical_incremental_mesh_update(pt, vs, fs,
                                                     vertex_weight=vert_weight,
                                                     max_angle=max_angle)


# view the mesh
graphics.mayavi_addMesh(mfig, vs, fs, representation='surface',
                        scalars=vert_weight, colormap='viridis', color=None)
graphics.mayavi_addPoint(mfig, pt)
