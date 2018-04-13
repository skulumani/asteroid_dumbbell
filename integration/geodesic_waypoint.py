"""Test out creating and plotting a geodesic waypoint (great circle) on the 
sphere
"""
from point_cloud import wavefront
from kinematics import sphere
from visualization import graphics

from lib import geodesic, surface_mesh

import numpy as np

# generate a sphere for plotting
sphere_mesh = surface_mesh.SurfMesh(1, 1, 1, 10, 0.15, 0.5)
vs, fs = sphere_mesh.verts(), sphere_mesh.faces()

# create two random points on the sphere
initial_point_cartesian = sphere.rand(2);
final_point_cartesian = sphere.rand(2);

# compute waypoints inbetween
waypoints_cartesian = geodesic.sphere_waypoint(initial_point_cartesian, final_point_cartesian, 5)

# plot everythign on a mayavi figure
mfig = graphics.mayavi_figure()
graphics.mayavi_axes(mfig, [-1, 1, -1, 1, -1, 1])
graphics.mayavi_addMesh(mfig, vs, fs, color=(0, 0, 1), opacity=0.2)
graphics.mayavi_points3d(mfig, waypoints_cartesian, color=(0,0, 1))
graphics.mayavi_addPoint(mfig, initial_point_cartesian, color=(0, 1, 0))
graphics.mayavi_addPoint(mfig, final_point_cartesian, color=(1, 0, 0))
