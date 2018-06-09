from dynamics import asteroid as asteroid_python
from lib import asteroid as asteroid_cpp
from lib import mesh_data
from point_cloud import wavefront
import numpy as np


v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')
ast_p = asteroid_python.Asteroid('itokawa', 0, 'obj')

mesh = mesh_data.MeshData(v, f)
ast_c = asteroid_cpp.Asteroid('itokawa', mesh)

state = np.array([1, 2, 3])

ast_p.polyhedron_potential(state)
ast_c.polyhedron_potential(state)
