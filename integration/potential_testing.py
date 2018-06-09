from dynamics import asteroid as asteroid_python
from lib import asteroid as asteroid_cpp
from lib import mesh_data
from point_cloud import wavefront
import numpy as np

ast_p = asteroid_python.Asteroid('castalia', 0, 'obj')

v, f = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj')
mesh = mesh_data.MeshData(v, f)
ast_c = asteroid_cpp.Asteroid('castalia', mesh)

state = np.array([1, 2, 3])

ast_p.polyhedron_potential(state)
ast_c.polyhedron_potential(state)
