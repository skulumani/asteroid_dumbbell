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


def read_obj_points(filename, pointSource=vtk.vtkProgrammableSource):
    r"""A source filter for VTK to read OBJ files

    This is a programmable filter to allow for the input of OBJ formatted shape
    files into a VTK object.

    Parameters
    ----------
    filename : string
        <`4:Description of the variable`>

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

    fname = open('./data/point_clouds/cactus.3337.pts')

    line = fname.readline()
    while line:
        data = line.split()
        if data and data[0] == 'p':
            x, y, z = float(data[1]), float(data[2]), float(data[3])
            points.InsertNextPoint(x, y, z)
        line = fname.readline()
