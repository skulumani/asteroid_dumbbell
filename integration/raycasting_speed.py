"""Compare speed of raycasting with Python vs. Python C++ Bindings

Here we compare C++ raycasting to Python/VTK raycasting
Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""

from point_cloud import wavefront, raycaster
from lib import cgal, mesh_data
import numpy as np

pos = np.array([5, 0, 0])
target = np.array([0, 0, 0])

filename = './data/shape_model/ITOKAWA/itokawa_very_high.obj'
polydata = wavefront.read_obj_to_polydata(filename)
caster = raycaster.RayCaster(polydata, flag='bsp')
intersection = caster.castray(pos, target)

v, f = wavefront.read_obj(filename)
mesh = mesh_data.MeshData(v, f)
caster_cpp = cgal.RayCaster(mesh)
intersection_cpp = caster_cpp.castray(pos, target)

