"""Wavefront OBJ module

Handle reading and writing to OBJ files. This will convert a wavefront file
to/from a numpy array (point cloud).

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
import numpy as np
import pdb
import vtk

# TODO: Add documentation and link to OBJ format
def write_obj(verts, faces, filename, comments=False):
    """Given numpy arrays of vertices/faces this will write it out to a OBJ file

    Assumes triangular faces and the numpy indexing defines the topology
    """
    with open(filename, 'w') as fname:
        if comments:
            fname.write('# Created by Shankar Kulumani - use at your own risk')
            fname.write('# List of vertices\n')

        for v in verts:
            fname.write('v {} {} {}\n'.format(v[0], v[1], v[2]))

        for f in faces:
            fname.write('f {} {} {}\n'.format(f[0]+1, f[1]+1, f[2]+1))

    return 0

# TODO: Add documentation and link to OBJ format
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
                faces.append([(int(val)-1) for val in value.split()])

    faces = np.asarray(faces)
    verts = np.asarray(verts) 
    return verts, faces

def create_points(vertices):
    """Given a (n, 3) numpy array of vertices this will place each one inot the 
    vtkPoints object
    """
    pass

