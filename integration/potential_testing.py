from dynamics import asteroid as asteroid_python
from lib import asteroid as asteroid_cpp
from lib import mesh_data
from point_cloud import wavefront
import numpy as np

ast_p = asteroid_python.Asteroid('castalia', 0, 'obj')

v, f = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj')
mesh_data = mesh_data.MeshData(v, f)
mesh_param = asteroid_cpp.MeshParam(mesh_data)
ast_c = asteroid_cpp.Asteroid('castalia', mesh_param)

state = np.array([1, 2, 3])

ast_p.polyhedron_potential(state)
ast_c.polyhedron_potential(state)
