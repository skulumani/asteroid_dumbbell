# compare the 'uniform' mesh from distmesh and cgal
from visualization import graphics
from point_cloud import wavefront
from lib import surface_mesh

from kinematics import attitude
import numpy as np
import scipy.io

import pdb

# load data from sphere and ellipsoid
sphere_data = scipy.io.loadmat('./data/sphere_distmesh.mat')
ellipsoid_data = scipy.io.loadmat('./data/ellipsoid_distmesh.mat')

vs, fs = sphere_data['v'], sphere_data['f']
ve, fe = ellipsoid_data['v'], ellipsoid_data['f']

# now use cgal
sphere = surface_mesh.SurfMesh(0.5, 0.5, 0.5, 10, 0.015, 0.5)
ellipsoid = surface_mesh.SurfMesh(1, 2, 3, 10, 0.06, 0.5)

print("Some statisitics")
print("Distmesh Sphere: Vertices: {} Faces: {}".format(vs.shape[0], fs.shape[0]))
print("Distmesh Ellipsoid: Vertices: {} Faces: {}".format(ve.shape[0], fe.shape[0]))
print("SurfMesh Sphere: Vertices: {} Faces: {}".format(sphere.verts().shape[0], sphere.faces().shape[0]))
print("SurfMesh Ellipsoid: Vertices: {} Faces: {}".format(ellipsoid.verts().shape[0], ellipsoid.faces().shape[0]))

mfig_distmesh = graphics.mayavi_figure()
mfig_surfmesh = graphics.mayavi_figure()
mfig_distmesh_ellipsoid = graphics.mayavi_figure()
mfig_surfmesh_ellipsoid = graphics.mayavi_figure()

graphics.mayavi_addMesh(mfig_distmesh, vs, fs, representation='wireframe')
graphics.mayavi_addTitle(mfig_distmesh, "Distmesh", color=(0, 0, 0))
graphics.mayavi_addMesh(mfig_surfmesh, sphere.verts(), sphere.faces(), representation='wireframe')
graphics.mayavi_addTitle(mfig_surfmesh, "CGAL Surface Mesh generation", color=(0, 0, 0))

graphics.mayavi_addMesh(mfig_distmesh_ellipsoid, ve, fe, representation='wireframe')
graphics.mayavi_addTitle(mfig_distmesh_ellipsoid, "Distmesh", color=(0, 0, 0))
graphics.mayavi_addMesh(mfig_surfmesh_ellipsoid, ellipsoid.verts(), ellipsoid.faces(), representation='wireframe')
graphics.mayavi_addTitle(mfig_surfmesh_ellipsoid, "CGAL Surface Mesh generation", color=(0, 0, 0))

