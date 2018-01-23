"""Some speed/profile comparisions for testing out searching
"""
import numpy as np
from point_cloud import wavefront
import utilities
import pdb
import scipy.io
import timeit

def polyhedron_setup():

    # load mat file and use a smaller number of faces
    # mat = scipy.io.loadmat('./dynamics/ITOKAWA/itokawa_model.mat')
    # f = mat['F_32'] - 1
    # v = mat['V_32']

    # define a polyhedron
    v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')
    # v, f = wavefront.read_obj('./integration/cube.obj')
    # v, f = wavefront.read_obj('./integration/triangle.obj')

    num_v = v.shape[0]
    num_f = f.shape[0]
    num_e = 3 * (num_v - 2)

    (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3,
     e1_vertex_map, e2_vertex_map, e3_vertex_map, 
     normal_face, e1_normal, e2_normal,e3_normal, center_face, e_vertex_map, unique_index) = wavefront.polyhedron_parameters(v, f)

    return (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3, e1_vertex_map, e2_vertex_map,
    e3_vertex_map, normal_face, e1_normal, e2_normal,e3_normal, center_face)

def edge_searching(e1, e2, e3):

    # this is very slow
    (e1_ind1b, e1_ind2b, e1_ind3b,
    e2_ind1b, e2_ind2b, e2_ind3b,
    e3_ind1b, e3_ind2b, e3_ind3b) = wavefront.search_edge(e1, e2, e3)

    return 0

def edge_map_searching(e1_vertex_map, e2_vertex_map, e3_vertex_map):
    # this should be faster
    (e1_ind1b_new, e1_ind2b_new, e1_ind3b_new,
    e2_ind1b_new, e2_ind2b_new, e2_ind3b_new,
    e3_ind1b_new, e3_ind2b_new, e3_ind3b_new) = wavefront.search_edge_vertex_map(e1_vertex_map, e2_vertex_map, e3_vertex_map)

    return 0

def edge_search_time(num_runs=1000, repeat=3):
    """Compute time for using older searching over edges
    """
    TEST_CODE='''
edge_searching(e1, e2, e3)
'''

    SETUP_CODE='''
from __main__ import polyhedron_setup, edge_searching
(Fa, Fb, Fc, V1, V2, V3, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map, normal_face, e1_normal, e2_normal,e3_normal, center_face) = polyhedron_setup()
'''

    times=timeit.repeat(setup=SETUP_CODE,
                        stmt=TEST_CODE,
                        repeat=repeat,
                        number=num_runs)
    print('Searching over edges: {} ms +/- {} ms'.format(np.mean(times)/ num_runs * 1e3, np.std(times)/num_runs * 1e3))

def edge_map_search_time(num_runs=1000, repeat=3):
    """Compute time for using new hotness searching over vertex maps
    """
    TEST_CODE='''
edge_map_searching(e1_vertex_map, e2_vertex_map, e3_vertex_map)
'''

    SETUP_CODE='''
from __main__ import polyhedron_setup, edge_map_searching
(Fa, Fb, Fc, V1, V2, V3, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map, normal_face, e1_normal, e2_normal,e3_normal, center_face) = polyhedron_setup()
'''

    times=timeit.repeat(setup=SETUP_CODE,
                        stmt=TEST_CODE,
                        repeat=repeat,
                        number=num_runs)
    
    print('Searching over edge map: {} ms +/- {} ms'.format(np.mean(times)/ num_runs * 1e3, np.std(times)/num_runs * 1e3))

def example_searching_array():
    """Example from

    https://stackoverflow.com/questions/8251541/numpy-for-every-element-in-one-array-find-the-index-in-another-array

    Find indices of y that are in x
    """
    x = np.array([3, 5, 7, 1, 9, 8, 6, 6])
    y = np.array([2, 1, 5, 10, 100, 6])
    print('x: {}'.format(x))
    print('y: {}'.format(y))
    index = np.argsort(x)
    sorted_x = x[index]
    sorted_index = np.searchsorted(sorted_x, y)

    yindex = np.take(index, sorted_index, mode='clip')
    mask = x[yindex] != y
    result = np.ma.array(yindex, mask=mask)
    print(result)
    # xe = np.outer([1,] * len(x), y)
    # ye = np.outer(x, [1,] * len(y))
    xe = np.broadcast_to(y, (len(x), len(y))) 
    ye = np.broadcast_to(x, (len(y), len(x))).T
    indx, indy = np.where(np.equal(xe, ye))
    print(indy)

    # create index of matches
    # indices that is the size of x which shows the equal value in y
    index_x = np.full(x.shape,-1, dtype='int')
    index_y = np.full(y.shape,-1, dtype='int')
    index_x[indx] = indy 
    index_y[indy] = indx

    return index_x, index_y

if __name__ == "__main__":
    edge_search_time(1, 1)
    edge_map_search_time(1, 1)

