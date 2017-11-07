"""Some speed/profile comparisions for testing out searching
"""
import numpy as np
from point_cloud import wavefront
import utilities

# define a polyhedron
v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')

num_v = v.shape[0]
num_f = f.shape[0]
num_e = 3 * (num_v - 2)

(Fa, Fb, Fc, V1, V2, V3, e1, e2, e3, e1_vertex_map, e2_vertex_map,
e3_vertex_map, normal_face, e1_normal, e2_normal,e3_normal, center_face) = wavefront.polyhedron_parameters(v, f)

# add face/edge map

e1_index_ismember = utilities.ismember_index(-e1, e1)

# duplicate this by searchinf over e1_vertex_map instead
