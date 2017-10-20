"""Wavefront OBJ module

Handle reading and writing to OBJ files. This will convert a wavefront file
to/from a numpy array (point cloud).

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
import numpy as np
import pdb

def write_obj():
    pass

def read_obj(filename):
    """Read the OBJ file and save to a numpy array
    """
    faces = []
    verts = []
    with open(filename, 'r') as f:
        for line in f:
            prefix, value = line.split(' ', 1)

            if prefix == 'v':
                verts.append([float(val) for val in value.split()]) 
            elif prefix == 'f':
                faces.append([int(val) for val in value.split()])

    faces = np.asarray(faces)
    verts = np.asarray(verts) 
    return faces, verts
