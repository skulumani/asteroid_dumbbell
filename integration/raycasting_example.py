"""Script to do some raycasting with the simulated lidar
"""
from kinematics import attitude
from point_cloud import wavefront, raycaster
import numpy as np
from visualization import graphics
import pdb

filename = './data/shape_model/CASTALIA/castalia.obj'
polydata = wavefront.read_obj_to_polydata(filename)

caster_obb = raycaster.RayCaster(polydata, flag='obb')
caster_bsp = raycaster.RayCaster(polydata, flag='bsp')

sensor = raycaster.Lidar(view_axis=np.array([1, 0, 0]), num_step=3)

# need to translate the sensor and give it a pointing direction
pos = np.array([5, 0, 0])
dist = 5 # distance for each raycast
R = attitude.rot3(np.pi/2)

# find the inersections
# targets = pos + sensor.rotate_fov(R) * dist
targets = sensor.define_targets(pos, R, dist)
intersections_obb = caster_obb.castarray(pos, targets)
intersections_bsp = caster_bsp.castarray(pos, targets)

# plot
fig = graphics.mayavi_figure(bg=(1, 1, 1))
graphics.mayavi_addPoly(fig, polydata)

graphics.mayavi_addPoint(fig, pos, radius=0.05)

# draw lines from pos to all targets
for pt in targets:
    graphics.mayavi_addLine(fig, pos, pt, color=(1, 0, 0))

# draw a point at all intersections
# for ints in intersections_obb:
#     graphics.mayavi_addPoint(fig, ints, radius=0.01, color=(0, 1, 0))

for ints in intersections_bsp:
    graphics.mayavi_addPoint(fig, ints, radius=0.01, color=(1, 1, 0))

graphics.mlab.orientation_axes(figure=fig)
# graphics.mlab.text3d(0, 0, 0.1, 'Green: OBBTree', figure=fig, scale=0.1)
# graphics.mlab.text3d(0, 0, -0.1, 'Yellow: BSPTree', figure=fig, scale=0.1)
