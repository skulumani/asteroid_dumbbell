"""Convert point clouds to meshes using VTK

Extended description of the module

Notes
-----
    This is an example of an indented section. It's like any other section,
    but the body is indented to help it stand out from surrounding text.

If a section is indented, then a section break is created by
resuming unindented text.

Attributes
----------
module_level_variable1 : int
    Descrption of the variable

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""

import numpy as np
import vtk
import pdb


def read_obj_points(pointSource, filename):
    r"""A source filter for VTK to read OBJ files

    This is a programmable filter to allow for the input of OBJ formatted shape
    files into a VTK object.

    Parameters
    ----------
    filename : string
        Name of the wavefront OBJ file to input
    pointSource : vtkProgrammableSource
        pointSource = vtk.ProgrammableSource
        This is the vtk object that we'll use to store the points

    Returns
    -------
    none : none

    Raises
    ------
    none
        Because you shouldn't have done that.

    See Also
    --------
    vtk_examples.py - lots of examples from the tutorials

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------
    Cite the relevant literature, e.g. [1]_.  You may also cite these
    references in the notes section above.

    .. [1] Add link to VTK example

    """

    output = pointSource.GetPolyDataOutput()
    points = vtk.vtkPoints()
    output.SetPoints(points)
    pdb.set_trace() 
    # open the OBJ file
    with open(filename) as fname:
        line = fname.readline()
        while line:
            data = line.split()
            if data and data[0] == 'v':
                x, y, z = float(data[1]), float(data[2]), float(data[3])
                points.InsertNextPoint(x, y, z)
            line = fname.readline()

def read_numpy_points(pointSource, point_cloud):
    """Read point cloud from an array
    """
    output = pointSource.GetPolyDataOutput()
    points = vtk.vtkPoints()
    output.SetPoints(points)
    # open the OBJ file
    for ii in range(0, point_cloud.shape[0]):
        vector = point_cloud[ii, :]
        x, y, z = vector
        points.InsertNextPoint(x, y, z)
