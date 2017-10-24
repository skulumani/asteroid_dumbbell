"""Wavefront OBJ module

Handle reading and writing to OBJ files. This will convert a wavefront file
to/from a numpy array (point cloud).

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
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
from vtk.util import numpy_support
import pdb
from point_cloud import wavefront

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

def obj_mesh_comparison(filename='./data/itokawa_low.obj'):
    """Compare the mesh from the OBJ file and a vtk reconstruction
    """

    pointSource = vtk.vtkProgrammableSource()
    filename='./data/itokawa_low.obj'

    def read_obj_points():
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
        # open the OBJ file
        with open(filename) as fname:
            line = fname.readline()
            while line:
                data = line.split()
                if data and data[0] == 'v':
                    x, y, z = float(data[1]), float(data[2]), float(data[3])
                    points.InsertNextPoint(x, y, z)

                line = fname.readline()

    pointSource.SetExecuteMethod(read_obj_points)

    surf = vtk.vtkSurfaceReconstructionFilter()
    surf.SetInputConnection(pointSource.GetOutputPort())

    cf = vtk.vtkContourFilter()
    cf.SetInputConnection(surf.GetOutputPort())
    cf.SetValue(0, 0.0)
    
    reverse = vtk.vtkReverseSense()
    reverse.SetInputConnection(cf.GetOutputPort())
    reverse.ReverseCellsOn()
    reverse.ReverseNormalsOn()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reverse.GetOutputPort())

    surfaceActor = vtk.vtkActor()
    surfaceActor.SetMapper(mapper)
    surfaceActor.GetProperty().SetDiffuseColor(1, 0.3882, 0.2784)
    surfaceActor.GetProperty().SetSpecularColor(1, 1, 1)
    surfaceActor.GetProperty().SetSpecular(0.4)
    surfaceActor.GetProperty().SetSpecularPower(50)

    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    ren.AddActor(surfaceActor)
    ren.SetBackground(1, 1, 1)
    renWin.SetSize(400, 400)
    ren.GetActiveCamera().SetFocalPoint(0, 0, 0)
    ren.GetActiveCamera().SetPosition(1, 0, 0)
    ren.GetActiveCamera().SetViewUp(0, 0, 1)
    
    ren.ResetCamera()
    ren.GetActiveCamera().Azimuth(20)
    ren.GetActiveCamera().Elevation(30)
    ren.GetActiveCamera().Dolly(1.2)
    ren.ResetCameraClippingRange()

    iren.Initialize()
    renWin.Render()
    iren.Start()

    # write the new surface to a wavefront file OBJ
    objWriter = vtk.vtkOBJExporter()
    objWriter.SetFilePrefix('./data/point_clouds/itokawa_low_vtkwriter')
    objWriter.SetRenderWindow(renWin)
    objWriter.Write()

def reduced_mesh_generation(filename='./data/itokawa_low.obj', step=1):
    """Read in the OBJ shape file, degrade the number of vectors and then 
    create a surface
    """

    pointSource = vtk.vtkProgrammableSource()
    filename='./data/itokawa_low.obj'
    faces, verts = wavefront.read_obj(filename)

    def read_numpy_points():
        r"""A source filter for VTK to read Numpy point cloud

        This is a programmable filter to allow for the input of numpy point cloud
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
        # open the OBJ file
        for ii in range(0, verts.shape[0], step):
            points.InsertNextPoint(verts[ii, 0], verts[ii, 1], verts[ii, 2])

    pointSource.SetExecuteMethod(read_numpy_points)

    surf = vtk.vtkSurfaceReconstructionFilter()
    surf.SetInputConnection(pointSource.GetOutputPort())

    cf = vtk.vtkContourFilter()
    cf.SetInputConnection(surf.GetOutputPort())
    cf.SetValue(0, 0.0)
    
    reverse = vtk.vtkReverseSense()
    reverse.SetInputConnection(cf.GetOutputPort())
    reverse.ReverseCellsOn()
    reverse.ReverseNormalsOn()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reverse.GetOutputPort())

    surfaceActor = vtk.vtkActor()
    surfaceActor.SetMapper(mapper)
    surfaceActor.GetProperty().SetColor(0.5, 0.5, 0.5)
    surfaceActor.GetProperty().SetDiffuseColor(1, 1, 1)
    surfaceActor.GetProperty().SetSpecularColor(1, 1, 1)
    surfaceActor.GetProperty().SetSpecular(0.4)
    surfaceActor.GetProperty().SetSpecularPower(50)

    ren = vtk.vtkRenderer()
    ren.AddActor(surfaceActor)
    ren.SetBackground(0, 0, 0)
    ren.GetActiveCamera().SetFocalPoint(0, 0, 0)
    ren.GetActiveCamera().SetPosition(1, 0, 0)
    ren.GetActiveCamera().SetViewUp(0, 0, 1)
    
    ren.ResetCamera()
    ren.GetActiveCamera().Azimuth(20)
    ren.GetActiveCamera().Elevation(30)
    ren.GetActiveCamera().Dolly(1.2)
    ren.ResetCameraClippingRange()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    renWin.SetSize(800,800)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    iren.Initialize()
    renWin.Render()
    iren.Start()

# TODO: Test writing to obj files by comparing to wavefront
def write_vtk_obj(renWin, filename):
    """Given a render window we'll write it to a OBJ file
    """
    # write the new surface to a wavefront file OBJ
    objWriter = vtk.vtkOBJExporter()
    objWriter.SetFilePrefix(filename)
    objWriter.SetRenderWindow(renWin)
    objWriter.Write()

# TODO: write another function to convert directly to numpy arrays and test
def read_obj_to_polydata(filename):
    """Test to read and display a wavefront OBJ

    """
        
    reader = vtk.vtkOBJReader()
    reader.SetFileName(filename)
    reader.Update()
    polyData = reader.GetOutput()
    return polyData

def make_vtk_idlist(array):
    """Turn an iterable listing vertices in a face into a VTK list
    """
    vil = vtk.vtkIdList()
    for i in array:
        vil.InsertNextId(int(i))
    return vil

def numpy_to_vtk_poly(vertices, faces):
    r"""Convert numpy to vtk poly data

    Extended description of the function.

    Parameters
    ----------
    var1 : array_like and type
        <`4:Description of the variable`>

    Returns
    -------
    describe : type
        Explanation of return value named describe

    Other Parameters
    ----------------
    only_seldom_used_keywords : type
        Explanation of this parameter

    Raises
    ------
    BadException
        Because you shouldn't have done that.

    See Also
    --------
    other_func: Other function that this one might call

    Notes
    -----
    You may include some math:

    .. math:: X(e^{j\omega } ) = x(n)e^{ - j\omega n}

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------
    Cite the relevant literature, e.g. [1]_.  You may also cite these
    references in the notes section above.

    .. [1] Shannon, Claude E. "Communication theory of secrecy systems."
    Bell Labs Technical Journal 28.4 (1949): 656-715

    Examples
    --------
    An example of how to use the function

    >>> a = [1, 2, 3]
    >>> print [x + 3 for x in a]
    [4, 5, 6]
    >>> print "a\n\nb"
    a
    b

    """ 
    # initialize some of the objects
    body = vtk.vtkPolyData()
    points = vtk.vtkPoints()
    polys = vtk.vtkCellArray()
    
    # load the data
    for ii in range(vertices.shape[0]):
        points.InsertPoint(ii, vertices[ii])
    
    for ii in range(faces.shape[0]):
        polys.InsertNextCell(make_vtk_idlist(faces[ii]))

    body.SetPoints(points)
    body.SetPolys(polys)

    return body

# TODO: Add documentation
def vtk_poly_to_numpy(polyData):
    """Convert a vtkPolyData object to the vertices and faces

    """
    # output = polyData.GetOutput()
    number_of_triangles = polyData.GetNumberOfCells()
    faces = np.zeros((number_of_triangles, 3), dtype=int)

    for ii in range(number_of_triangles):
        triangle = polyData.GetCell(ii)
        faces[ii, :] = np.array([triangle.GetPointId(x) for x in range(3)])

    # get the vertices
    vertices = numpy_support.vtk_to_numpy(polyData.GetPoints().GetData()) 
    # get the faces
    
    return vertices, faces

# TODO: Add a decimate function which takes/outputs numpy arrays and uses VTK
def decimate_polydata(polyData, num_faces):
    """Take a polydata and decimate it and return another polydata
    """
    pass

def decimate_numpy(vertices, faces, num_faces):
    r"""Reduce the size of a numpy polyhedron using VTK

    Extended description of the function.

    Parameters
    ----------
    var1 : array_like and type
        <`4:Description of the variable`>

    Returns
    -------
    describe : type
        Explanation of return value named describe

    Other Parameters
    ----------------
    only_seldom_used_keywords : type
        Explanation of this parameter

    Raises
    ------
    BadException
        Because you shouldn't have done that.

    See Also
    --------
    other_func: Other function that this one might call

    Notes
    -----
    You may include some math:

    .. math:: X(e^{j\omega } ) = x(n)e^{ - j\omega n}

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------
    Cite the relevant literature, e.g. [1]_.  You may also cite these
    references in the notes section above.

    .. [1] Shannon, Claude E. "Communication theory of secrecy systems."
    Bell Labs Technical Journal 28.4 (1949): 656-715

    Examples
    --------
    An example of how to use the function

    >>> a = [1, 2, 3]
    >>> print [x + 3 for x in a]
    [4, 5, 6]
    >>> print "a\n\nb"
    a
    b

    """ 
    
    # reduction target as a fraction
    scale = num_faces/faces.shape[0]
    polyhedron = numpy_to_vtk_poly(vertices, faces)

    # decimate
    decimate = vtk.vtkDecimatePro()
    decimate.SetInputData(polyhedron)
    decimate.SetTargetReduction(scale)
    decimate.Update()

    decimatedPoly = vtk.vtkPolyData()
    decimatedPoly.ShallowCopy(decimate.GetOutput())

    # extract out the points from a vtkPolyData

    # return faces/vertices
