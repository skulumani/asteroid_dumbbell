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
import logging
from multiprocessing import Pool
from collections import namedtuple
import warnings
import pdb

import numpy as np
import vtk
from vtk.util import numpy_support

import utilities

warnings.filterwarnings(action="ignore", category=RuntimeWarning,
                        message=r"All-NaN")
logger = logging.getLogger(__name__)

# named tuple to hold all the parameters (pre-computed) for a mesh
MESH_PARAM = namedtuple('MESH_PARAM', ['Fa', 'Fb', 'Fc', 'V1', 'V2', 'V3',
                                       'e1', 'e2', 'e3', 'e1_vertex_map',
                                       'e2_vertex_map', 'e3_vertex_map',
                                       'normal_face', 'e1_normal', 'e2_normal',
                                       'e3_normal', 'center_face',
                                       'e_vertex_map', 'unique_index',
                                       'edge_vertex_map', 'edge_face_map',
                                       'vertex_face_map', 'e1_face_map',
                                       'e2_face_map', 'e3_face_map',
                                       'e1_ind1b', 'e1_ind2b', 'e1_ind3b',
                                       'e2_ind1b', 'e2_ind2b', 'e2_ind3b',
                                       'e3_ind1b', 'e3_ind2b', 'e3_ind3b'])

# TODO: Create better function names

def write_obj(verts, faces, filename, comments=False):
    r"""Output vertices and faces to the Wavefront OBJ format

    Given a shape model defined in terms of numpy arrays, this will output them
    to a OBJ file

    Parameters
    ----------
    verts : numpy array v x 3 
        Arrays defining the vertices of the shape in the body fixed frame
    faces : numpy array f x 3
        Array defining the toplogy of the shape. Each row defines the vertices
        which make up that face.  It is assumed they are numbered in a right
        hand sense, such that the normal to each face is outward pointing
    filename : string
        Filename and path to save the OBJ file. This file will be overwritten
    comments : boolean
        Can write comments to the OBJ file. Not implemented yet

    Returns
    -------
    out : integer
        Just a 0 if successful. No other error codes

    See Also
    --------
    Look for OBJ format in ./docs/obj_format.txt

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

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

def read_obj(filename):
    r"""Read a OBJ shape model and output vertices and faces

    This will read a shape model and store the values into numpy arrays.

    Parameters
    ----------
    filename : string
        Name of OBJ file to read

    Returns
    -------
    verts : numpy array v x 3
        Array of vertices - each row is a vector in the body fixed frame
    faces : numpy array f x 3
        Mapping of the vectors which define each face. It is already corrected to use
        zero based indexing and each vector is numbered assuming a right hand orientation.

    See Also
    --------
    write_obj : inverse function which writes OBJ files

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
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

def ellipsoid_mesh(a, b, c, density=20, subdivisions=1):
    r"""Ellipsoid Mesh model

    verts, faces = ellipsoid_mesh(a, b, c, density)

    Parameters
    ----------
    a, b, c : floats
        Semi axes lengths (x, y, z ) directions respectively in kilometer
    density : int
        Density for the sperical coordinate parameterization
    subdivisions : int
        subdivisions for the mesh

    Returns
    -------
    verts : (V, 3)
        Vertices of the ellipsoid
    faces : (F, 3)
        Faces of the ellipsoid

    See Also
    --------
    reconstruct_numpy : VTK surface reconstruction from vertices
    mesh_subdivide : use this to subdivide the mesh into more faces
    
    Notes
    -----
    The density and subdivision cause differences in the number of vertices
    and faces.
    here is some help

    Density          Subdivisions           # vertices              # faces
    10                  0                       110                 216
    20                  0                       254                 504
    30                  0                       434                 864
    10                  1                       434                 864
    20                  1                       1010                 2016
    30                  1                       1730                 3456
    10                  2                       1730                 3456
    20                  2                       4034                 8064
    30                  2                       6914                 13824
    
    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    # define spherical angles
    theta = np.linspace(-np.pi/2, np.pi/2, num=density)
    phi = np.linspace(-np.pi, np.pi, num=density)
    
    tg, pg = np.meshgrid(theta, phi)
    # compute the vector
    x = a * np.cos(tg) * np.cos(pg)
    y = b * np.cos(tg) * np.sin(pg)
    z = c * np.sin(tg)

    # add to a list
    v = np.stack((x.ravel(), y.ravel(), z.ravel()), axis=-1)
    
    # now need to do surface reconstruction to get the faces
    verts, faces = reconstruct_numpy(v)
    verts, faces = mesh_subdivide(verts, faces, subdivisions, 'loop')

    return verts, faces

# TODO : Write this function
def create_points(vertices):
    """Given a (n, 3) numpy array of vertices this will place each one inot the 
    vtkPoints object
    """
    pass

# TODO: Add documentation
def read_numpy_points(pointSource, point_cloud):
    """Read point cloud from an array
    """
    output = pointSource.GetPolyDataOutput()
    points = vtk.vtkPoints()
    output.SetPoints(points)
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

def reconstruct_numpy(verts):
    r"""Surface Reconstruction using VTK

    v, f = reconstruct_numpy(verts)

    Parameters
    ----------
    verts : (n, 3) array
        Array holding all of the vertices of a point cloud

    Returns
    -------
    v : (V, 3) array
        Array of all the vertices of the mesh
    f : (F, 3) array
        Array of triangular facets defining the mesh. 

    Notes
    -----
    This is the same format used by the wavefront OBJ files
    
    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    pointSource = vtk.vtkProgrammableSource()

    def read_numpy_points():
        output = pointSource.GetPolyDataOutput()
        points = vtk.vtkPoints()
        output.SetPoints(points)
        for ii in range(0, verts.shape[0],1):
            points.InsertNextPoint(verts[ii, 0], verts[ii, 1], verts[ii, 2])

    pointSource.SetExecuteMethod(read_numpy_points)

    surf = vtk.vtkSurfaceReconstructionFilter()
    surf.SetNeighborhoodSize(10) # a low value makes a craggly surface
    surf.SetInputConnection(pointSource.GetOutputPort())
    
    cf = vtk.vtkContourFilter()
    cf.SetInputConnection(surf.GetOutputPort())
    cf.SetValue(0, 0.0)
    
    reverse = vtk.vtkReverseSense()
    reverse.SetInputConnection(cf.GetOutputPort())
    reverse.ReverseCellsOn()
    reverse.ReverseNormalsOn()
    reverse.Update()
    # convert to polydata
    poly = vtk.vtkPolyData()
    poly.ShallowCopy(reverse.GetOutput())
    # convert to numpy arrays
    v_new, f_new = polydatatomesh(poly)

    return v_new, f_new

def polydata_subdivide(polydata, subdivisions=1, style='loop'):
    r"""Subdivide a polydata

    poly_out = polydata_subdivide(polydata, subdivisions, style)

    Parameters
    ----------
    polydata : vtkPolyData
        The polydata to subdivide
    subdivisions : int
        Number of subdivisions. New faces will be 4**subdivision
    loop : string
        Loop or Butterfly method to subdivide

    Returns
    -------
    polydata : vtkPolyData
        The subdivided polydata

    See Also
    --------
    mesh_subdivide : Subdivide a numpy array mesh

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    if style=='loop':
        logger.info('Loop Subdivision')
        subdivider = vtk.vtkLoopSubdivisionFilter()
    elif style=='butterfly':
        logger.info('Butterfly Subdivision')
        subdivider = vtk.vtkButterflySubdivisionFilter()

    subdivider.SetNumberOfSubdivisions(subdivisions)
    subdivider.SetInputData(polydata)
    subdivider.Update()
    poly_output = vtk.vtkPolyData()
    poly_output.ShallowCopy(subdivider.GetOutput())
    return poly_output

def mesh_subdivide(v, f, subdivisions=1, style='loop'):
    r"""Subdivide a numpy mesh array

    nv, nf = mesh_subdivide(v, f, subdivisions, style)

    Parameters
    ----------
    v : numpy array (n, 3)
        Vertices
    f : numpy array (f, 3)
        Faces 
    subdivisions : int
        Number of subdivisions. New faces will be 4**subdivision
    loop : string
        Loop or Butterfly method to subdivide

    Returns
    -------
    nv : numpy array (n, 3)
        Vertices
    nf : numpy array (f, 3)
        Faces 

    See Also
    --------
    polydata_subdivide : Subdivide a polydata object

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """

    # convert mesh to poly data
    polydata = meshtopolydata(v, f)

    # do the subdivision on the polydata
    poly_subdivide = polydata_subdivide(polydata, subdivisions, style)

    # convert back to mesh and output
    return polydatatomesh(poly_subdivide)

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

# TODO: Add documentaiton
def write_render_window(renWin, filename):
    """Given a render window we'll write it to a OBJ file
    """
    # write the new surface to a wavefront file OBJ
    objWriter = vtk.vtkOBJExporter()
    objWriter.SetFilePrefix(filename)
    objWriter.SetRenderWindow(renWin)
    objWriter.Write()
    return 0

# TODO: Add documentation
def write_vtkPolyData(polyData, filename):
    """Write a polyData to a OBJ file
    """
    # create the render window then output that

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polyData)

    # connect the mapper to an actor
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # render the cube actor
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.0, 0.0, 0.0)
    renderer.AddActor(actor)

    # create a render window
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)

    write_render_window(render_window, filename)
    return 0

# TODO: write another function to convert directly to numpy arrays and test
def read_obj_to_polydata(filename):
    """Test to read and display a wavefront OBJ

    """
        
    reader = vtk.vtkOBJReader()
    reader.SetFileName(filename)
    reader.Update()
    polyData = reader.GetOutput()
    return polyData

# TODO: Add documentation
def read_obj_to_numpy(filename):
    """Use VTK to convert a OBJ file directly to a numpy array
    """
    polyData = read_obj_to_polydata(filename)
    v, f = polydatatomesh(polyData)
    return v, f

def make_vtk_idlist(array):
    """Turn an iterable listing vertices in a face into a VTK list
    """
    vil = vtk.vtkIdList()
    for i in array:
        vil.InsertNextId(int(i))
    return vil

def meshtopolydata(vertices, faces):
    r"""Convert numpy to vtk poly data

    polydata = meshtopolydata(v, f)

    Parameters
    ----------
    vertices : numpy array (v, 3)
        Array definining all the vertices in the body fixed frame
    faces : numpy array (f, 3)
        Topological connection between the vertices

    Returns
    -------
    polydata : vtk.vtkPolyData 
        vtk polydata object defining the polyhedron

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    # initialize some of the objects
    polydata = vtk.vtkPolyData()
    points = vtk.vtkPoints()
    polys = vtk.vtkCellArray()
    
    # load the data
    for ii in range(vertices.shape[0]):
        points.InsertPoint(ii, vertices[ii])
    
    for ii in range(faces.shape[0]):
        polys.InsertNextCell(make_vtk_idlist(faces[ii]))

    polydata.SetPoints(points)
    polydata.SetPolys(polys)

    return polydata 

def polydatatomesh(polyData):
    r"""Convert VTK Polydata to numpy arrays V, F

    v, f = polydatatomesh(polyData)

    Parameters
    ----------
    polyData : vtk.vtkPolyData
        Polyhedron object from vtk

    Returns
    -------
    vertices : numpy array (v, 3)
        All the vertices associated with the polyhedron
    faces : numpy array (f, 3)
        Connection between the vertices in the polyhedron

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
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

# TODO: Add documentation
def decimate_polydata(polyData, ratio=0.5):
    """Take a polydata and decimate it and return another polydata
    """
    # decimate
    decimate = vtk.vtkDecimatePro()
    decimate.SetInputData(polyData)
    decimate.SetTargetReduction(ratio)
    decimate.Update()

    decimatedPoly = vtk.vtkPolyData()
    decimatedPoly.ShallowCopy(decimate.GetOutput())
    
    return decimatedPoly

# TODO: Add documentation
# TODO: Research other settings - https://www.vtk.org/doc/nightly/html/classvtkDecimatePro.html
def decimate_numpy(vertices, faces, ratio=0.5, preserve_topology=True,
                   preserve_boundary=False, splitting=False):
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
    polyhedron = meshtopolydata(vertices, faces)

    # decimate
    decimate = vtk.vtkDecimatePro()
    decimate.SetInputData(polyhedron)
    decimate.SetTargetReduction(ratio)
    # TODO: Mention that this will prevent you from reaching the target number of faces
    if preserve_topology:
        decimate.PreserveTopologyOn()
    else:
        decimate.PreserveTopologyOff()
    
    if preserve_boundary:
        decimate.BoundaryVertexDeletionOff()
    else:
        decimate.BoundaryVertexDeletionOn()

    if splitting:
        decimate.SplittingOn()
    else:
        decimate.SplittingOff()

    decimate.Update()

    decimatedPoly = vtk.vtkPolyData()
    decimatedPoly.ShallowCopy(decimate.GetOutput())

    # extract out the points from a vtkPolyData
    dec_vertices, dec_faces = polydatatomesh(decimatedPoly)

    # return faces/vertices
    return dec_vertices, dec_faces

def vertex_face_map(V, F):
    r"""Create mapping between vertices and the connected faces

    vf_map = vertex_face_map(V, F)

    Parameters
    ----------
    V : numpy array (# vertices, 3)
        The list of vertices defining the mesh
    F : numpy array (# faces, 3)
        Connection between the vertices to define the triangular surface

    Returns
    -------
    vf_map : list object of lists (# verts, )
        Each element represents the indices of faces that are composed of that 
        vertex.
        For example, vf_map[0] = list([0, 1, 2]) means that vertex zero, ie.
        V[0, :] is used to define faces 0, 1, 2. Or equivalently that each of the 
        following F[[0, 1, 2], :] contains a 0 somewhere in that row.

    See Also
    --------
    polyhedron_parameters : another function to compute lots of useful stuff

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    vertex_face_map = [list() for _ in range(V.shape[0])]
    # loop through F
    for face,verts in enumerate(F):
        for v in verts:
            vertex_face_map[v].append(face)
     
    return vertex_face_map

def normal_face(V, F):
    r"""Compute the normal to each face

    normal_face = normal_face(V, F)

    Parameters
    ----------
    V : numpy array (v, 3)
        Array defining all the vertices of the mesh
    F : numpy array (f, 3)
        Array defining the topology of the mesh. Each row has vertex indices
        (zero based) which define the face

    Returns
    -------
    normal_face : (f, 3)
        The normal vector (unit) to each face

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    num_v = V.shape[0]
    num_f = F.shape[0]
    num_e = 3 * (num_v - 2)

    # calculate all the edges - zero indexing for python
    Fa = F[:, 0] 
    Fb = F[:, 1]
    Fc = F[:, 2]

    V1 = V[Fa, :]
    V2 = V[Fb, :]
    V3 = V[Fc, :]
    
    # Get all edge vectors
    e1 = V2 - V1
    e2 = V3 - V2
    e3 = V1 - V3

    # normal to face
    normal_face = np.cross(e1, e2)
    normal_face = normal_face / \
        np.tile(np.reshape(
            np.sqrt(np.sum(normal_face**2, axis=1)), (num_f, 1)), (1, 3))

    return normal_face

def center_of_face(V, F):
    r"""Find center of each face of mesh

    Extended description of the function.

    Parameters
    ----------
    V : numpy array (v, 3)
        Array defining all the vertices of the mesh
    F : numpy array (f, 3)
        Array defining the topology of the mesh. Each row has vertex indices
        (zero based) which define the face

    Returns
    -------
    cof : numpy array (f, 3)
        The center of each face of the mesh.

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    num_v = V.shape[0]
    num_f = F.shape[0]
    num_e = 3 * (num_v - 2)

    # calculate all the edges - zero indexing for python
    Fa = F[:, 0] 
    Fb = F[:, 1]
    Fc = F[:, 2]

    V1 = V[Fa, :]
    V2 = V[Fb, :]
    V3 = V[Fc, :]

    cof = (V1 + V2 + V3) / 3

    return cof

# TODO: Add unit testing
def polyhedron_parameters(V, F):
    r"""Compute parameters associated with a polyhedron and useful for
    calculating the gravity model

    This takes the shape model (vertices, and faces) and computes a bunch of
    elements which are needed later in the gravity model.

    Parameters
    ----------
    V : numpy array (v, 3)
        Array defining all of the vertices in the shape model
    F : numpy array (f, 3)
        Array defining the vertices composed in each face in the model

    Returns
    -------
    Fa, Fb, Fc : numpy array (f,)
        All of the 1, 2, 3 vertices in each face, respectively
        Fa = F[:, 0], Fb = F[:, 1], Fc = F[:, 2]
    V1, V2, V3 : numpy array (f, 3)
        All of vertices associated with Fa, Fb, Fc respectively
        V1 = V[Fa, :], V2 = V[Fb, :], V3 = V[Fc, :] 
    e1, e2, e3 : numpy array (f, 3)
        These are each of the e1, e2, e3 edges on each face.
        e1 = V2 - V1, e2 = V3 - V2, e3 = V1 - V3
    e1_vertex_map, e2_vertex_map, e3_vertex_map : numpy array (f, 2)
        These define the vertices which make up each edge
        e1_vertex_map = [Fb, Fa], etc.
    normal_face : numpy array (f, 3)
        Normal vector to each face. Assumes outward facing normal. so is a function 
        of the vertex numbering
    e1_normal, e2_normal,e3_normal : numpy array (f, 3)
        This is the normal to each edge, and again is outward facing. 
        Cross product of face normal and edge vector
    center_face : numpy array (f, 3)
        The geometric center for each face
    e_vertex_map : numpy array (e, 2)
        Unique vertices which define all the edges (no duplicates) 
        
    See Also
    --------
    asteroid.Asteroid : Polyhedron potential model class

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------

    .. [1] WERNER, Robert A. "The Gravitational Potential of a Homogeneous
    Polyhedron or Don't Cut Corners". Celestial Mechanics and Dynamical
    Astronomy. 1994, vol 59
    """ 
    # calculate shape parameters

    num_v = V.shape[0]
    num_f = F.shape[0]
    num_e = 3 * (num_v - 2)
    # calculate all the edges - zero indexing for python
    Fa = F[:, 0] 
    Fb = F[:, 1]
    Fc = F[:, 2]

    V1 = V[Fa, :]
    V2 = V[Fb, :]
    V3 = V[Fc, :]
    
    # Get all edge vectors
    e1 = V2 - V1
    e2 = V3 - V2
    e3 = V1 - V3

    e1_vertex_map = np.vstack((Fb, Fa)).T
    e2_vertex_map = np.vstack((Fc, Fb)).T
    e3_vertex_map = np.vstack((Fa, Fc)).T
     
    e_vertex_map, unique_index = np.unique(np.sort(np.vstack((e1_vertex_map,
                                                              e2_vertex_map,
                                                              e3_vertex_map)),
                                                   axis=1),
                                           axis=0,
                                           return_index=True)

    # Normalize edge vectors
    # e1_norm=e1./repmat(sqrt(e1(:,1).^2+e1(:,2).^2+e1(:,3).^2),1,3);
    # e2_norm=e2./repmat(sqrt(e2(:,1).^2+e2(:,2).^2+e2(:,3).^2),1,3);
    # e3_norm=e3./repmat(sqrt(e3(:,1).^2+e3(:,2).^2+e3(:,3).^2),1,3);

    # normal to face
    normal_face = np.cross(e1, e2)
    normal_face = normal_face / \
        np.tile(np.reshape(
            np.sqrt(np.sum(normal_face**2, axis=1)), (num_f, 1)), (1, 3))

    # normal to each edge
    e1_normal = np.cross(e1, normal_face)
    e1_normal = e1_normal / \
        np.tile(np.reshape(
            np.sqrt(np.sum(e1_normal**2, axis=1)), (num_f, 1)), (1, 3))

    e2_normal = np.cross(e2, normal_face)
    e2_normal = e2_normal / \
        np.tile(np.reshape(
            np.sqrt(np.sum(e2_normal**2, axis=1)), (num_f, 1)), (1, 3))

    e3_normal = np.cross(e3, normal_face)
    e3_normal = e3_normal / \
        np.tile(np.reshape(
            np.sqrt(np.sum(e3_normal**2, axis=1)), (num_f, 1)), (1, 3))

    # calculate the center of each face
    center_face = 1.0 / 3 * (V[Fa, :] + V[Fb, :] + V[Fc, :])

    # Calculate Angle of face seen from vertices
    # Angle =  [acos(dot(e1_norm',-e3_norm'));acos(dot(e2_norm',-e1_norm'));acos(dot(e3_norm',-e2_norm'))]';
    
    edge_vertex_map = (e1_vertex_map, e2_vertex_map, e3_vertex_map)
    vf_map = vertex_face_map(V, F)

    (e1_ind1b, e1_ind2b, e1_ind3b,
    e2_ind1b, e2_ind2b, e2_ind3b,
    e3_ind1b, e3_ind2b, e3_ind3b) = search_edge_vertex_map(e1_vertex_map,
                                                                        e2_vertex_map, 
                                                                        e3_vertex_map)
    # build the edge face maps
    e1_face_map, e2_face_map, e3_face_map = build_edge_face_map(e1_ind1b, e1_ind2b, e1_ind3b,
                                                                            e2_ind1b, e2_ind2b, e2_ind3b,
                                                                            e3_ind1b, e3_ind2b, e3_ind3b)
    
    edge_face_map = (e1_face_map, e2_face_map, e3_face_map)

    mesh_parameters = MESH_PARAM(Fa=Fa, Fb=Fb, Fc=Fc, V1=V1, V2=V2, V3=V3,
                                 e1=e1, e2=e2, e3=e3,
                                 e1_vertex_map=e1_vertex_map,
                                 e2_vertex_map=e2_vertex_map,
                                 e3_vertex_map=e3_vertex_map,
                                 normal_face=normal_face, e1_normal=e1_normal,
                                 e2_normal=e2_normal, e3_normal=e3_normal,
                                 center_face=center_face,
                                 e_vertex_map=e_vertex_map,
                                 unique_index=unique_index,
                                 edge_vertex_map=edge_vertex_map,vertex_face_map=vf_map,
                                 e1_face_map=e1_face_map, e2_face_map=e2_face_map,
                                 e3_face_map=e3_face_map, edge_face_map=edge_face_map,
                                 e1_ind1b=e1_ind1b, e1_ind2b=e1_ind2b, e1_ind3b=e1_ind3b,
                                 e2_ind1b=e2_ind1b, e2_ind2b=e2_ind2b, e2_ind3b=e2_ind3b,
                                 e3_ind1b=e3_ind1b, e3_ind2b=e3_ind2b, e3_ind3b=e3_ind3b)

    return mesh_parameters

def search_edge(e1, e2, e3):
    r"""Search for matching edges by looking directly at teh computed edges,
    rather than the edge/vertex maps

    Given all of the edges, this will find the opposite edge somewhere else and
    provide the index of that match. One caveat is that this method will fail
    for parallel edges. In a closed polyhedron there should only be a single match
    in all of the edges.

    Parameters
    ----------
    e1 : numpy array (f, 3)
        Array defining all of the e1 (others respecitvely) edges in the polyhedron

    Returns
    -------
    e1_ind1b : numpy array (f,)
        Defines the matching edges for each edge -e1 that is also in e1.
        If there is no match then that row in -1. So assume e1_ind1b[0] = 2 then e1[0, :] == - e1[2, :]
        and e1_vertex_map[0,0] == e1_vertex_map[2,1]
    e1_ind2b, e1_ind3b, e2_ind1b, e2_ind2b, e2_ind3b, e3_ind1b, e3_ind2b, e3_ind3b
        These are all similar as previously defined. Just defines the matching (inverse) 
        edges and where they're located.

    See Also
    --------
    search_edge_vertex_map : Use this improved and faster function
    ismember_index : a kludge implementation of Matlab ismember

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    e1_ind1b = utilities.ismember_index(-e1, e1)
    e1_ind2b = utilities.ismember_index(-e1, e2)
    e1_ind3b = utilities.ismember_index(-e1, e3)

    e2_ind1b = utilities.ismember_index(-e2, e1)
    e2_ind2b = utilities.ismember_index(-e2, e2)
    e2_ind3b = utilities.ismember_index(-e2, e3)

    e3_ind1b = utilities.ismember_index(-e3, e1)
    e3_ind2b = utilities.ismember_index(-e3, e2)
    e3_ind3b = utilities.ismember_index(-e3, e3)

    return (e1_ind1b, e1_ind2b, e1_ind3b,
            e2_ind1b, e2_ind2b, e2_ind3b,
            e3_ind1b, e3_ind2b, e3_ind3b)

def vertex_map_search(arrays):
    r"""Finds locations of matching/inverse elements in edge/vertex maps

    Given arrays which hold the indices of vertices for each edge, this function
    will find the opposite edge in the other array.

    Parameters
    ----------
    arrays : tuple (2,)
        arrays[0] - a_map numpy array (f, 2) which defines the vertices involved in all the a edges
        arrays[1] - b_map numpy array (f, 2) which defines the vertices involved in all the b edges 

    Returns
    -------
    index_map : numpy array (f,)
        This is an array which has an integer (non-negative) which defines the
        index (in b_map) which is the opposite edge as a_map. 

    See Also
    --------
    vertex_map_inverse : finds the opposite match without having to do an
    intensive search by inverting the mapping 
    utilities.search_index : does the searching between two arrays

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    a_map = arrays[0]
    b_map = arrays[1]
    invalid = -1
    num_e = a_map.shape[0]
    a = 0
    b = 1

    index_map = np.full(num_e, invalid, dtype='int')
    inda1, indb1 = utilities.search_index(a_map[:, a], b_map[:, b])
    
    index_match = np.where(a_map[inda1, b] == b_map[indb1, a])
    index_map[inda1[index_match]] = indb1[index_match]
    return index_map

def vertex_map_inverse(a_map, invalid=-1):
    r"""Create the inverse mapping for vertices and edges

    Given a list of matching edges of a which also exist in b (a_map). This function
    will give you a list of matching edges of b which are in a (a_map).

    Parameters
    ----------
    a_map : numpy array (f,)
        This is a list of matches of a_map (and the edges it's defining) to some other array of edges.
    invalid : int
        The non-matches are signified with invalid

    Returns
    -------
    b_map : numpy array (f,)
        This the inverse of a_map and shows the matches of b in a and their location

    See Also
    --------
    vertex_map_search : this finds the original mapping

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    a_loc = np.where(a_map != invalid)
    b_loc = a_map[a_loc]
    b_map = np.full(a_map.shape, invalid, dtype='int')
    b_map[b_loc] = a_loc
    return b_map

def search_edge_vertex_map(e1_vertex_map, e2_vertex_map, e3_vertex_map):
    r"""Find matching edges across the vertex/edge maps

    Instead of searching through the actual edges, this function will find opposite 
    edges that exist by searching through the lists of vertices/edges.
    
    Each face is composed of three edges, and each edge is a member of two faces.
    Each face has a e1, e2, and e3 edge and this function will find the location
    of the opposite edge.

    Parameters
    ----------
    e1_vertex_map : numpy array f x 2
        e1 edges are defined as V[e1_vertex_map[:, 0], :] - V[e1_vertex_map[:, 1], :]
    e2_vertex_map : numpy array f x 2
        e2 edges are defined as V[e2_vertex_map[:, 0], :] - V[e2_vertex_map[:, 1], :]
    e3_vertex_map : numpy array f x 2
        e3 edges are defined as V[e3_vertex_map[:, 0], :] - V[e3_vertex_map[:, 1], :]
        
    Returns
    -------
    e1_ind1b : numpy array (f,)
        Defines the matching edges for each edge -e1 that is also in e1.
        If there is no match then that row in -1. So assume e1_ind1b[0] = 2 then e1[0, :] == - e1[2, :]
        and e1_vertex_map[0,0] == e1_vertex_map[2,1]
    e1_ind2b, e1_ind3b, e2_ind1b, e2_ind2b, e2_ind3b, e3_ind1b, e3_ind2b, e3_ind3b
        These are all similar as previously defined. Just defines the matching (inverse) 
        edges and where they're located.

    See Also
    --------
    vertex_map_search : actually does the search for matching values in two arrays
    vertex_map_inverse : if we know matches of a in b, we can easily get the matches of b in a

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    invalid = -1
    num_e = e1_vertex_map.shape[0]
    a = 0
    b = 1

    ######################## e1 searching #####################################
    with Pool(6) as p:
        # func = partial(vertex_map_search, e1_vertex_map)
        index_list = p.map(vertex_map_search, ((e1_vertex_map, e1_vertex_map),
                                               (e1_vertex_map, e2_vertex_map),
                                               (e1_vertex_map, e3_vertex_map),
                                               (e2_vertex_map, e2_vertex_map),
                                               (e2_vertex_map, e3_vertex_map),
                                               (e3_vertex_map, e3_vertex_map)))

    e1_ind1b =index_list[0]
    e1_ind2b =index_list[1]
    e1_ind3b =index_list[2]

    e2_ind1b = vertex_map_inverse(e1_ind2b)
    e2_ind2b = index_list[3]
    e2_ind3b = index_list[4]

    e3_ind1b = vertex_map_inverse(e1_ind3b)
    e3_ind2b = vertex_map_inverse(e2_ind3b)
    e3_ind3b = index_list[5]

    
    return (e1_ind1b, e1_ind2b, e1_ind3b,
            e2_ind1b, e2_ind2b, e2_ind3b,
            e3_ind1b, e3_ind2b, e3_ind3b)

# TODO Add documentation for this function and the face map
def build_edge_face_map(e1_ind1b, e1_ind2b, e1_ind3b,
                        e2_ind1b, e2_ind2b, e2_ind3b,
                        e3_ind1b, e3_ind2b, e3_ind3b):

    faces_list = np.arange(e1_ind1b.shape[0])
    e1_face_map = np.stack((faces_list, e1_ind1b, e1_ind2b, e1_ind3b), axis=1)
    e2_face_map = np.stack((faces_list, e2_ind1b, e2_ind2b, e2_ind3b), axis=1)
    e3_face_map = np.stack((faces_list, e3_ind1b, e3_ind2b, e3_ind3b), axis=1)

    return e1_face_map, e2_face_map, e3_face_map

# TODO: Maybe make this into a map function 
def compute_edge_dyad(e1_face_map, e2_face_map, e3_face_map,
                      e1_normal, e2_normal, e3_normal,
                      normal_face, invalid=-1):

    edge_normals = np.stack((e1_normal, e2_normal, e3_normal), axis=2)
    # create a big list of edge normals for e1 edges
    row, col = np.where(e1_face_map[:, 1:] != invalid)
    e1_adjacent_face_index = e1_face_map[row, col+1]
    e1_adjacent_face = col
    e1_matching_normal = edge_normals[e1_adjacent_face_index, :, e1_adjacent_face]

    e1_adjacent_normal_face = normal_face[e1_adjacent_face_index, :]

    E1_edge = np.einsum('ij,ik->jki', normal_face, e1_normal) + np.einsum('ij,ik->jki', e1_adjacent_normal_face, e1_matching_normal)
    # create a big list of edge normals for e2 edges
    row, col = np.where(e2_face_map[:, 1:] != invalid)
    e2_adjacent_face_index = e2_face_map[row, col+1]
    e2_adjacent_face = col
    e2_matching_normal = edge_normals[e2_adjacent_face_index, :, e2_adjacent_face]
    e2_adjacent_normal_face = normal_face[e2_adjacent_face_index, :]

    E2_edge = np.einsum('ij,ik->jki', normal_face, e2_normal) + np.einsum('ij,ik->jki', e2_adjacent_normal_face, e2_matching_normal)
    # create a big list of edge normals for e3 edges
    row, col = np.where(e3_face_map[:, 1:] != invalid)
    e3_adjacent_face_index = e3_face_map[row, col+1]
    e3_adjacent_face = col
    e3_matching_normal = edge_normals[e3_adjacent_face_index, :, e3_adjacent_face]
    e3_adjacent_normal_face = normal_face[e3_adjacent_face_index, :]

    E3_edge = np.einsum('ij,ik->jki', normal_face, e3_normal) + np.einsum('ij,ik->jki', e3_adjacent_normal_face, e3_matching_normal)
    
    # compute the dyad
    return E1_edge, E2_edge, E3_edge

def face_dyad_loop(normal_face):

    F_face= np.zeros([3, 3, normal_face.shape[0]])
    for ii in range(normal_face.shape[0]):
        F_face[:, :, ii] = np.outer(
            normal_face[ii, :], normal_face[ii, :])

    return F_face

def edge_dyad_loop(e1_face_map, e2_face_map, e3_face_map,
                   e1_normal, e2_normal, e3_normal,
                   normal_face, invalid=-1):
    num_f = e1_face_map.shape[0]
    E1_edge = np.zeros([3, 3, num_f])
    E2_edge = np.zeros([3, 3, num_f])
    E3_edge = np.zeros([3, 3, num_f])
    
    for ii in range(num_f):

        # find the edge normals for all edges of the current face
        # also pull out the edge normals for each adjacent face (3 adjacent
        # faces
        nA1 = e1_normal[e1_face_map[ii, 0], :]
        nA2 = e2_normal[e2_face_map[ii, 0], :]
        nA3 = e3_normal[e3_face_map[ii, 0], :]

        # find adjacent face for edge 1
        col = np.where(e1_face_map[ii, 1:] != invalid)[0][0]
        face_index = e1_face_map[ii, col + 1]

        if col == 0:  # adjacent face is also edge 1
            nB1 = e1_normal[face_index, :]
        elif col == 1:  # adjacent face is edge 2
            nB1 = e2_normal[face_index, :]
        elif col == 2:
            nB1 = e3_normal[face_index, :]

        nA = normal_face[ii, :]
        nB = normal_face[face_index, :]

        # second order dyadic tensor
        E1_edge[:, :, ii] = np.outer(nA, nA1) + np.outer(nB, nB1)

        # find adjacent face for edge 2
        col = np.where(e2_face_map[ii, 1:] != invalid)[0][0]
        face_index = e2_face_map[ii, col + 1]

        if col == 0:  # adjacent face is also edge 1
            nB2 = e1_normal[face_index, :]
        elif col == 1:  # adjacent face is edge 2
            nB2 = e2_normal[face_index, :]
        elif col == 2:
            nB2 = e3_normal[face_index, :]

        nB = normal_face[face_index, :]

        # second order dyadic tensor
        E2_edge[:, :, ii] = np.outer(nA, nA2) + np.outer(nB, nB2)

        # find adjacent face for edge 3
        col = np.where(e3_face_map[ii, 1:] != invalid)[0][0]
        face_index = e3_face_map[ii, col + 1]

        if col == 0:  # adjacent face is also edge 1
            nB3 = e1_normal[face_index, :]
        elif col == 1:  # adjacent face is edge 2
            nB3 = e2_normal[face_index, :]
        elif col == 2:
            nB3 = e3_normal[face_index, :]

        nB = normal_face[face_index, :]

        # second order dyadic tensor
        E3_edge[:, :, ii] = np.outer(nA, nA3) + np.outer(nB, nB3)

    return E1_edge, E2_edge, E3_edge

# TODO Figure out which version is fastest( sum vs einsum)
def dist_array(pt, array):
    r"""Distance between a point and an array of points

    dist, ind = dist_array(pt, array)

    Parameters
    ----------
    pt : numpy array (n, )
        Point to check
    array : numpy array (m, n)
        Array of points, each row is a n-dimensional vector

    Returns
    -------
    dist : float
        Euclidean distance between pt and the minimum point in array
    ind : int
        Array index (row location) of the minimum distance piont in array

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    # dist = np.sum((pt - array)**2, axis=1)
    delta = array - pt
    dist = np.einsum('ij,ij->i', delta, delta)
    ind = np.where(dist == np.nanmin(dist))[0]
    return np.sqrt(dist[ind]), ind

def sign_of_largest(array):
    r"""Return sign of largest element of array

    sgn = sign_of_largest(array)

    Parameters
    ----------
    array : numpy array (n,)
        input array. Should be one dimensional

    Returns
    -------
    sgn : int
        Sign of largest element. Sign will either be 1 or -1. If largest 
        element is zero then sgn = 1

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    index = np.argmax(np.absolute(array))
    sgn = np.sign(array[index])
    if sgn == 0:
        sgn = 1
    return sgn

def mesh_incremental_update(pt, v, f, method='all'):
    r"""Incorporate pt into the mesh defined by v,f

    v_new, f_new = mesh_incremental_update(pt, v, f, method='all')

    Parameters
    ----------
    pt : numpy array (3,)
        Point to check in 3D
    v : numpy array (v, 3)
        Vertices defining the mesh
    f : numpy array (f, 3)
        Topological connection of mesh
    method : string
        Type of method to use
        'all' : Can add as a vertex, face, or edge
        'vertex' : Only modify the vertex and never add a vertex

    Returns
    -------
    v : numpy array (v, 3)
        New vertices
    f : numpy array (f, 3)
        New faces

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    mesh_parameters = polyhedron_parameters(v, f)
    
    if method == 'all':
        D, P, V, E, F, primitive = distance_to_mesh(pt, v, f, mesh_parameters)


        if primitive == 'vertex':
            nv, nf = vertex_insertion(pt, v, f, D, P, V, E, F)
        elif primitive == 'edge':
            nv, nf = edge_insertion(pt, v, f, D, P, V, E, F)
        elif primitive == 'face':
            nv, nf = face_insertion(pt, v, f, D, P, V, E, F)
    elif method == 'vertex': # only check for closest vertex
        normal_face = mesh_parameters.normal_face
        edge_vertex_map = mesh_parameters.edge_vertex_map
        edge_face_map = mesh_parameters.edge_face_map
        vf_map = mesh_parameters.vertex_face_map
        

        D, P, V, E, F = distance_to_vertices(pt, v, f, normal_face,
                                             edge_vertex_map, edge_face_map,
                                             vf_map)

        # figure out the minimum and output that
        D, P, V , E , F  = distance_minimum(D, P, V, E, F)

        nv, nf = vertex_insertion(pt, v, f, D, P, V, E, F)

    return nv, nf

def radius_mesh_incremental_update(pt, v, f, mesh_parameters,
                                   max_angle=np.deg2rad(45),
                                   angle_std=2):
    r"""Update a mesh by radially moving vertices

    nv, nf = radius_mesh_incremental_update(pt, v, f)

    Parameters
    ----------
    pt : (3,) numpy array
        The point to incorporate into the mesh
    v : (# v, 3) numpy array
        The vertices of the initial mesh
    f : (# f, 3) numpy array
        The connections between the vertices to form triangular faces

    Returns
    -------
    nv : numpy array (v, 3)
        New vertices
    nf : numpy array (f, 3)
        New faces

    Notes
    -----
    This will find the angular seperation between all the vertices and the 
    candidate point.
    The one the lies the closest, in the same direction (angle is close to zero),
    will be modified radially.
    This means that the orthogonal projection of the point onto the canditidate
    vertex is found.
    The vertex is modified and the mesh is returned.

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    normal_face = mesh_parameters.normal_face
    edge_vertex_map = mesh_parameters.edge_vertex_map
    edge_face_map = mesh_parameters.edge_face_map
    vf_map = mesh_parameters.vertex_face_map
    # find minimum angular seperatiaon 
    cos_angle = np.dot(v, pt)/np.linalg.norm(v, axis=1)/np.linalg.norm(pt)
    # find the index of the point which lies inside of a threshold
    # 1 sigma mask (extra points)
    # mask_sigma = np.ma.masked_less(cos_angle, np.cos(np.deg2rad(np.rad2deg(max_angle) * angle_std)))
    mask = np.ma.masked_less(cos_angle, np.cos(max_angle))
    
    # now find index of minimum angle (closest to 1)
    ind_angle = np.nonzero(mask == np.max(mask))[0]
    # ind_angle_sigma = np.nonzero(mask_sigma == np.max(mask_sigma))[0]
    ind_angle_region = np.nonzero(mask)[0]
    
    if ind_angle.size: # some points satisfy the constraint
        logger.info("pt: {} is within {} deg of v:{}. Radially changing the vertex".format(pt, np.rad2deg(max_angle), v[ind_angle, :]))
        # TODO Think about changing all of these points by a given radius
        # a = - pt
        # b = v[ind_angle, :]
        # vertex_param = - np.dot(b, a) / np.linalg.norm(b, axis=1)**2

        # # this is the candidate new point
        # vertex_intersections = vertex_param[:, np.newaxis] * b
        
        # # now find the index of the smallest change (minimum radius change)
        # radius_change = np.linalg.norm(b - vertex_intersections, axis=1)
        # min_radius = np.nonzero(radius_change == np.min(radius_change))[0]
        # min_ind = ind_angle[min_radius]

        # nv = v.copy()
        # nv[min_ind, :] = vertex_intersections[min_radius, :]
        # nf = f.copy()

        # modify everything within the region of interest
        vertex_param_region = -np.dot(v[ind_angle_region, :], - pt) / np.linalg.norm(v[ind_angle_region, :], axis=1)**2
        vertex_intersections_region = vertex_param_region[:, np.newaxis] * v[ind_angle_region, :]
        nv = v.copy()
        nv[ind_angle_region, :] = vertex_intersections_region
        nf=f.copy()

    else: # no point lies within the angle constraint. Now we'll add a vertex
        # TODO Need assertions that the mesh remains topologically valid 
        # find closest edge and face
        De, Pe, Ve, Ee, Fe = distance_to_edges(pt, v, f, normal_face,
                                               edge_vertex_map, edge_face_map,
                                               vf_map)
        De, Pe, Ve, Ee, Fe = distance_minimum(De, Pe, Ve, Ee, Fe)

        Df, Pf, Vf, Ef, Ff = distance_to_faces(pt, v, f, normal_face,
                                               edge_vertex_map, edge_face_map,
                                               vf_map)
        Df, Pf, Vf, Ef, Ff = distance_minimum(Df, Pf, Vf, Ef, Ff)
        
        # if not near an edge or face (outside of edge or face)
        if De or Df:
            try:
                if Df < De: # add vertex by replacing a vertex
                    logger.info('Face is closest')
                    nv, nf = face_insertion(pt, v, f, Df, Pf, Vf, Ef, Ff)
                else:
                    logger.info('Edge is closest')
                    nv, nf = edge_insertion(pt, v, f, De, Pe, Ve, Ee, Fe)
            except IndexError as err:
                logger.error("Some kind of index error at pt = {}. Error = {}\nJust using the old v,f".format(pt, err))
                nv = v.copy()
                nf = f.copy()
        else:
            logger.info('Pt:{} is not in view. Skipping')
            nv = v.copy()
            nf = f.copy()
        # whichever is closest is used to add the vertex


    return nv, nf

def spherical_incremental_mesh_update(pt_spherical, vs_spherical, f,
                                      surf_area, factor=1, radius_factor=1):

    # surface area on a spherical area gives us a range of long and lat
    delta_lat, delta_lon = spherical_surface_area(pt_spherical, surf_area, factor=1)

    # find elements that lie close to the measurement (lat/lon searching col 1 and 2)

    diff_angle = vs_spherical[:, [1, 2]] - pt_spherical[np.newaxis, 1:2]
    valid_lat = np.absolute(diff_angle[:, 0]) < delta_lat
    valid_lon = np.absolute(diff_angle[:, 1]) < delta_lon
    
    # indices that are within the range
    region_index = np.intersect1d(np.nonzero(valid_lat)[0], np.nonzero(valid_lon)[0])
    mesh_region = vs_spherical[region_index,:]
    delta_sigma = spherical_distance(pt_spherical, mesh_region)

    # now compute new radii for those in mesh region
    radius_scale = radius_scale_factor(delta_sigma, 1)
    
    mesh_region[:, 0] = radius_scale * pt_spherical[0]
    
    nv_spherical = vs_spherical.copy()
    
    nv_spherical[region_index, :] = mesh_region

    return nv_spherical, f

def distance_to_mesh(pt, v, f, mesh_parameters):
    r"""Minimum distance to a mesh

    D, P, V, E, F = distance_to_mesh(pt, v, f, mesh_parameters)

    Parameters
    ----------
    pt : numpy array (3,)
        Point to check in 3D
    v : numpy array (v, 3)
        Vertices defining the mesh
    f : numpy array (f, 3)
        Topological connection of mesh
    mesh_parameters : named tuple
        Tuple output from polyhedron parameters

    Returns
    -------
    D : float
        Signed distance from pt to the closest edge (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie on the closest edge
    V : int
        The unique vertices in the closest edges. This is a list of locations 
        for v
    E : int array
        The vertices for each closest edge. edge = V[E[0],:] - V[E[1], :] 
    F : numpy array (m, )
        Indices of all the faces associated with the edge (in V)

    See Also
    --------
    polyhedron_parameters : computes all the parameters for a mesh
    distance_to_vertices : finds closest vertex in mesh
    distance_to_edges : finds closest edge in mesh
    distance_to_faces : finds the closest face in mesh

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    # compute or pass in the polyhedron parameters
    D_min = np.inf
    dist_funcs = (distance_to_vertices, distance_to_edges, distance_to_faces)
    
    normal_face = mesh_parameters.normal_face
    edge_vertex_map = mesh_parameters.edge_vertex_map
    edge_face_map = mesh_parameters.edge_face_map
    vf_map = mesh_parameters.vertex_face_map
    
    D_buffer = np.inf

    for primitive, dist_fun in zip(('vertex', 'edge', 'face'), dist_funcs):
        D, P, V, E, F = dist_fun(pt, v, f, normal_face, edge_vertex_map,
                                 edge_face_map, vf_map)

        # figure out the minimum and output that
        D_temp, P_temp, V_temp, E_temp, F_temp = distance_minimum(D, P, V, E, F)
        
        if D_temp < D_buffer: # new distance is less. Reset the minimum
            D_min = D_temp
            P_min = P_temp
            V_min = V_temp
            E_min = E_temp
            F_min = F_temp
            D_buffer = D_min
            primitive_buffer = primitive
        else:
            pass
        # check if less than what we've seen already
    
    return D_min, P_min, V_min, E_min, F_min, primitive_buffer

def distance_minimum(D, P, V, E, F):
    """Determine the minimum distance in case there or multiple

    Dm, Pm, Vm, Em, Fm = distance_minimum(D, P, V, E, F)

    Parameters
    ----------
    D : float
        Signed distance from pt to the closest edge (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie on the closest edge
    V : int
        The unique vertices in the closest edges. This is a list of locations 
        for v
    E : int array
        The vertices for each closest edge. edge = V[E[0],:] - V[E[1], :] 
    F : numpy array (m, )
        Indices of all the faces associated with the edge (in V)

    Returns
    -------
    D : float
        Signed distance from pt to the closest edge (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie on the closest edge
    V : int
        The unique vertices in the closest edges. This is a list of locations 
        for v
    E : int array
        The vertices for each closest edge. edge = V[E[0],:] - V[E[1], :] 
    F : numpy array (m, )
        Indices of all the faces associated with the edge (in V)

    See Also
    --------

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    # determine if scalar or array output (many pionts are equidistant)
    if D.size == 1: # scalar closest point
        D_min = np.absolute(D)
        P_min = P
        V_min = V
        E_min = E
        F_min = F
    elif D.size > 1: # multiple minimum points
        ind = np.argmin(np.absolute(D))
        D_min = np.absolute(D[ind])
        P_min = P[ind]
        V_min = V[ind]
        E_min = E[ind]
        F_min = F[ind]
    elif D.size == 0: # empty minimum point
        D_min = []
        P_min = []
        V_min = []
        E_min = []
        F_min = []

    return D_min, P_min, V_min, E_min, F_min

def vertex_insertion(pt, v, f, D, P, V, E, F):
    r"""Insert a vertex into a mesh

    nv, nf = vertex_insertion(pt, v, f, D, P, V, E, F)

    Parameters
    ----------
    pt : (3,) numpy array
        The point to incorporate into the mesh
    v : (# verts, 3) numpy array
        The current vertices of the mesh
    f : (# faces, 3) numpy array
        Current faces of the mesh
    D : float
        Signed distance from pt to the closest edge (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie on the closest edge
    V : int
        The unique vertices in the closest edges. This is a list of locations 
        for v
    E : int array
        The vertices for each closest edge. edge = V[E[0],:] - V[E[1], :] 
    F : numpy array (m, )
        Indices of all the faces associated with the edge (in V)

    Returns
    -------
    nv : (# vertices , 3) numpy array
        The new vertices of the mesh
    nf : (# faces, 3) numpy array
        The new faces of the mesh

    See Also
    --------
    distance_minimum : find the minimum distance to the mesh

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    # based on closest distance insert the vertex
    new_v = v
    new_v[V,:] = pt

    new_f = f
    return new_v, new_f

def edge_insertion(pt, v, f, D, P, V, E, F):
    r"""Remove edge and insert a new vertex

    nv, nf = edge_insertion(pt, v, f, D, P, V, E, F)

    Parameters
    ----------
    pt : (3,) numpy array
        The point to incorporate into the mesh
    v : (# verts, 3) numpy array
        The current vertices of the mesh
    f : (# faces, 3) numpy array
        Current faces of the mesh
    D : float
        Signed distance from pt to the closest edge (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie on the closest edge
    V : int
        The unique vertices in the closest edges. This is a list of locations 
        for v
    E : int array
        The vertices for each closest edge. edge = V[E[0],:] - V[E[1], :] 
    F : numpy array (m, )
        Indices of all the faces associated with the edge (in V)

    Returns
    -------
    nv : (# vertices , 3) numpy array
        The new vertices of the mesh
    nf : (# faces, 3) numpy array
        The new faces of the mesh

    See Also
    --------
    distance_minimum : find the minimum distance to the mesh

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    nv = np.concatenate((v, pt[np.newaxis]))
    new_vertex_index = nv.shape[0]-1

    new_faces = []
    # find the vertices of the attached faces
    attached_vertices = f[F, :]
    # figure out which vertices are in the minimum edge
    face1_index = attached_vertices[0, :] == E[:, np.newaxis]
    face2_index = attached_vertices[1, :] == E[:, np.newaxis]


    # add two new faces associated with face 1
    for ii in range(2):
        nface = attached_vertices[0, :].copy()
        nface[face1_index[ii, :]] = new_vertex_index
        new_faces.append(nface)

    for ii in range(2):
        nface = attached_vertices[1, :].copy()
        nface[face2_index[ii, :]] = new_vertex_index
        new_faces.append(nface)

    nf = np.delete(f, F, axis=0)
    nf = np.concatenate((nf, np.array(new_faces)))
    
    return nv, nf

def face_insertion(pt, v, f, D, P, V, E, F):
    r"""Remove a face and add a new vertex

    nv, nf = face_insertion(pt, v, f, D, P, V, E, F)

    Parameters
    ----------
    pt : (3,) numpy array
        The point to incorporate into the mesh
    v : (# verts, 3) numpy array
        The current vertices of the mesh
    f : (# faces, 3) numpy array
        Current faces of the mesh
    D : float
        Signed distance from pt to the closest edge (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie on the closest edge
    V : int
        The unique vertices in the closest edges. This is a list of locations 
        for v
    E : int array
        The vertices for each closest edge. edge = V[E[0],:] - V[E[1], :] 
    F : numpy array (m, )
        Indices of all the faces associated with the edge (in V)

    Returns
    -------
    nv : (# vertices , 3) numpy array
        The new vertices of the mesh
    nf : (# faces, 3) numpy array
        The new faces of the mesh

    See Also
    --------
    distance_minimum : find the minimum distance to the mesh

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    nv = np.concatenate((v, pt[np.newaxis]))
    new_vertex_index = nv.shape[0]-1
    # find vertices of the face
    A, B, C = f[F, :]
    
    new_faces = np.array([[A, B, new_vertex_index],
                          [B, C, new_vertex_index],
                          [C, A, new_vertex_index]])

    nf = np.delete(f, F, axis=0)
    nf = np.concatenate((nf, np.array(new_faces)))
    return nv, nf

def distance_to_vertices(pt, v, f, normal_face, edge_vertex_map, 
                         edge_face_map,vf_map):
    r"""Find closest vertex in mesh to a given point

    D, P, F, V = distance_vertices(pt, v, f, normal_face)

    Parameters
    ----------
    pt : numpy array (3,)
        Point to check in 3D
    v : numpy array (v, 3)
        Vertices defining the mesh
    f : numpy array (f, 3)
        Topological connection of mesh
    normal_face : numpy array ( f, 3)
        Normal to the center of each face
    vf_map : numpy array (# v, m)
        This is a list object of lists which tells you the faces that contain
        any given vertex

    Returns
    -------
    D : float [3]
        Signed distance from pt to the closest vertex (+ outside, - inside)
    P : numpy array (n, 3)
        Location of the closest vertices. extracted from v
    V : int (n,)
        Index of the closest points in v. So P = v[V,:]. There may be many 
        points
    E : int
        Vertex elements for the edges. Each edge = v[E[0], :] - v[E[1], :]
    F : numpy array (m, q)
        Indices of all the faces associated with the vertices given in P

    See Also
    --------
    sign_of_largest : Finds the sign of largest element in array

    Notes
    -----
    This function may return many points if there are multiple vertices
    that are exactly the same distance. In such a case, the rows are defining
    a single vertex and the associated properties

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------

    .. [1] OROURKE, Joseph. Computational Geometry in C. 2 ed. Cambridge
    University Press, 1998.
    """
    # determine the vertex that is closest to pt
    dist, ind = dist_array(pt, v)
    P = np.squeeze(v[ind, :])
    V = ind
    D = []

    edges = np.concatenate(edge_vertex_map)
    if V.size > 1:
        E = [edges[np.where(edges == ii)[0],:] for ii in V]
        # determine the faces that are associated with any of the vertices in ind
        F = np.array([np.array(vf_map[ii]) for ii in ind])

        for ii, (faces, int_point) in enumerate(zip(F, P)):
            N = normal_face[faces, :]
            coeff = np.dot( N,pt - int_point)
            sign_of_value = sign_of_largest(coeff)
            D.append(dist[ii] * sign_of_value)
    else:
        E = edges[np.where(edges == V)[0], :]
        F = np.array(vf_map[int(ind)])

        N = normal_face[F, :]
        coeff = np.dot(N, pt - P)
        sign_of_value = sign_of_largest(coeff)
        D.append(dist * sign_of_value)

    assert (len(F) >= 1), "Vertex {} is not connected to any face.".format(ind)
    # for each vertex we need to compute the signed distance to its faces

    # TODO Better variables names
    return np.squeeze(D), np.squeeze(P), np.squeeze(V), np.squeeze(E), np.squeeze(F)

def distance_to_edges(pt, v, f, normal_face, edge_vertex_map,
                      edge_face_map, vf_map):
    r"""Compute distance to all edges and output minimum

    D, P, F, V = distance_to_edges(pt, v, f, normal_face,
                                   edge_vertex_map, edge_face_map)

    Parameters
    ----------
    pt : numpy array (3,)
        Point to check in 3D
    v : numpy array (v, 3)
        Vertices defining the mesh
    f : numpy array (f, 3)
        Topological connection of mesh
    normal_face : numpy array ( f, 3)
        Normal to the center of each face
    edge_vertex_map : 3 - tuple 
        (e1_vertex_map, e2_vertex_map, e3_vertex_map)
        Each element is size (# faces, 2). The elements represent the edges
        for each face. To find an edge you need to subtract the first column 
        minus the second column.
    edge_face_map : 3 - tuple
        (e1_face_map, e2_face_map, e3_face_map)
        Each element of tuple defines the mapping between the opposite edges.
        The size in each are (# faces, 4). The first column defines the face number.
        The last three columns give you the matching edge and the associated face.
        Columns show, e1, e2, e3 and rows are the face numbers.
    vf_map : numpy array (# v, m)
        This is a list object of lists which tells you the faces that contain
        any given vertex

    Returns
    -------
    D : float
        Signed distance from pt to the closest edge (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie on the closest edge
    V : int
        The unique vertices in the closest edges. This is a list of locations 
        for v
    E : int array
        The vertices for each closest edge. edge = V[E[0],:] - V[E[1], :] 
    F : numpy array (m, )
        Indices of all the faces associated with the edge (in V)

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    Notes
    -----
    Can automatically handle a single closest edge or multiple

    References
    ----------

    .. [1] OROURKE, Joseph. Computational Geometry in C. 2 ed. Cambridge
    University Press, 1998.
    """
    num_v = v.shape[0]
    num_f = f.shape[0]
    num_e = 3 * (num_v - 2)
    # put all the edges in an array

    # calculate all the edges - zero indexing for python
    Fa = f[:, 0] 
    Fb = f[:, 1]
    Fc = f[:, 2]

    e1_vertex_map, e2_vertex_map, e3_vertex_map = edge_vertex_map
    e1_face_map, e2_face_map, e3_face_map = edge_face_map

    edges = np.concatenate((e1_vertex_map, e2_vertex_map, e3_vertex_map))
    
    # find the parameteric intersection parameter for every edge (t)
    v1 = v[edges[:, 1], :]
    v2 = v[edges[:, 0], :]
    
    a = v1 - pt
    b = v2 - v1
    edge_param = - np.einsum('ij,ij->i', a, b) / np.linalg.norm(b, axis=1)**2
    
    # exclude intersections that are outside of the range 0, 1
    edge_param[edge_param <= 0] = np.nan

    mask = ~np.isnan(edge_param)
    mask[mask] &= edge_param[mask] >= 1
    edge_param[mask] = np.nan

    # find the distance between intersection (on edge) and point
    edge_intersections = v1 + edge_param[:, np.newaxis] * b
    dist, index = dist_array(pt, edge_intersections)
    P = edge_intersections[index, :]
    E = edges[index, :]
    V = E
    F = []
    # find associated faces for this edge (match the edge values
    for e_ind in E:
        va, vb = e_ind

        va_ind = np.where(f == va)[0]
        vb_ind = np.where(f == vb)[0]
        
        F.append(np.intersect1d(va_ind, vb_ind))

    N = []
    D = []

    # determine which edge for the given ind (closest)
    for ii, ind in enumerate(index):
        edge_face_map_row = edge_face_map[ind // num_f][ind % num_f, :]
        face_index = edge_face_map_row[edge_face_map_row != -1]
        face_normals = normal_face[face_index, :]

        N.append(face_normals)
        
        coeff = np.dot(face_normals, pt - P[ii, :])
        sign = sign_of_largest(coeff)
        D.append(dist[ii] * sign)
    
    return (np.squeeze(D), np.squeeze(P), np.squeeze(V), np.squeeze(E),
            np.squeeze(F))
            

def distance_to_faces(pt, v, f, normal_face, edge_vertex_map,
                      edge_face_map, vf_map):
    r"""Find distance to every face and output the closest one

    D, P, V, E, F = distance_to_faces(pt, V, F, normal_face)

    Parameters
    ----------
    pt : numpy array (3,)
        Point to check in 3D
    v : numpy array (v, 3)
        Vertices defining the mesh
    f : numpy array (f, 3)
        Topological connection of mesh
    normal_face : numpy array ( f, 3)
        Normal to the center of each face
         
    Returns
    -------
    D : float
        Signed distance from pt to the closest face (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie in the closest face
    F : int
        Index of the face that is closest
    V : numpy array (3, 3)
        Define the vertices of this closest face
        V = vertices[faces[F, :], :]

    Raises
    ------
    AssertionError : barycentric coordinates should sum to 1

    Notes
    -----
    This finds the distance from the pt to every face, then the minimum 
    distance is output.
    The algorithm projects each point onto the plane of each face, then 
    computes if the point lies inside the face or not.
    If it is not inside the face then it's not output.

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------

    .. [1] OROURKE, Joseph. Computational Geometry in C. 2 ed. Cambridge
    University Press, 1998. 

    """
    num_v = v.shape[0]
    num_f = f.shape[0]
    num_e = 3 * (num_v - 2)

    # extract out all the  vertices
    Fa, Fb, Fc = f[:, 0], f[:, 1], f[:, 2]
    v1, v2, v3 = v[Fa, :], v[Fb, :], v[Fc, :]

    # compute distance from pt to surface plane
    ptv1 = pt - v1
    dist = np.einsum('ij,ij->i', normal_face, ptv1)
    surf_intersections = pt - dist[:, np.newaxis] * normal_face

    # determine teh barycentric coordinates of each intersection point
    a = v2 - v1
    b = v3 - v1
    c = ptv1

    # form all the dot products
    adota = np.einsum('ij,ij->i', a, a)
    bdotb = np.einsum('ij,ij->i', b, b)
    adotb = np.einsum('ij,ij->i', a, b)
    cdota = np.einsum('ij,ij->i', c, a)
    cdotb = np.einsum('ij,ij->i', c, b)

    denom = adotb**2 - bdotb*adota
    s_param = (adotb * cdotb - bdotb * cdota) / denom
    t_param = (adotb *  cdota - adota * cdotb) / denom

    alpha = 1 - s_param - t_param
    beta = s_param
    gamma = t_param
    
    np.testing.assert_allclose(alpha + beta + gamma, 1,rtol=1e-3, err_msg='Barycentric coordinates are not valid') 

    barycentric = np.stack((alpha, beta, gamma), axis=1)
    # exclude intersections that don't lie inside the faces
    dist[np.absolute(dist) <= np.finfo(float).eps] = np.nan
    dist[np.any(barycentric < 0, axis=1)] = np.nan
    dist[np.any(barycentric >= 1, axis=1)] = np.nan

    # determine the closest face
    # TODO: Better variable names
    try:
        ind = np.where(np.absolute(dist) == np.nanmin(np.absolute(dist)))[0]
        D = dist[ind]
        P = surf_intersections[ind, :]
        V = [f[ii,:] for ii in ind]
        E=[np.stack((edge_vertex_map[0][ii], edge_vertex_map[1][ii],
                           edge_vertex_map[2][ii])) for ii in ind]
        F = ind

    except (ValueError,RuntimeWarning) as err:
        logger.warn('The point {} is not in view of any face: {}'.format(pt))

        D = P = F = V = []

    return np.squeeze(D), np.squeeze(P), np.squeeze(V), np.squeeze(E), np.squeeze(F)

def cartesian2spherical(vertices):
    r"""Convert cartesian to spherical coordinates  

    spherical = cartesian2spherical(vertices)

    Parameters
    ----------
    vertices: (v, 3)
        Array of vertices in body frame (x, y, z)

    Returns
    -------
    spherical : (v, 3) numpy array
        Spherical representation of vertices, (r, lat, long)

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    if len(vertices) == 3:
        r = np.sqrt(vertices[0]**2 + vertices[1]**2 + vertices[2]**2)
        longitude = np.arctan2(vertices[1], vertices[0])
        latitude = np.arcsin(vertices[2] / r)

        spherical = np.array([r,latitude,longitude])
    else:
        r = np.sqrt(vertices[:, 0]**2 + vertices[:, 1]**2 + vertices[:, 2]**2)
        longitude = np.arctan2(vertices[:, 1], vertices[:, 0])
        latitude = np.arcsin(vertices[:, 2] / r)
    
        spherical = np.stack((r, latitude, longitude), axis=1)
    return spherical

def spherical2cartesian(spherical):
    r"""Convert spherical to cartesian coordinates

    vertices = spherical2cartesian(spherical)

    Parameters
    ----------
    spherical : (v, 3)
        Spherical representation of vertices (r, lat, long)

    Returns
    -------
    vertices : (v, 3) numpy array
        Cartesian vertices [x, y, z]

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    if len(spherical) == 3:
        r, latitude, longitude = spherical[0], spherical[1], spherical[2]

        x = r * np.cos(latitude) * np.cos(longitude)
        y = r * np.cos(latitude) * np.sin(longitude)
        z = r * np.sin(latitude)

        vertices = np.array([x, y, z])
    else:
        r, latitude, longitude = spherical[:, 0], spherical[:, 1], spherical[:, 2]

        x = r * np.cos(latitude) * np.cos(longitude)
        y = r * np.cos(latitude) * np.sin(longitude)
        z = r * np.sin(latitude)

        vertices = np.stack((x, y, z), axis=1)

    return vertices

def spherical_surface_area(meas, surf_area, factor=1):
    """Find the range of latitutde and longitude given a desired surface area
    """
    r, lat, lon = meas
    del_angle = np.sqrt(surf_area/r**2 / np.cos(lat))

    delta_lat = del_angle;
    delta_lon = factor*delta_lat;

    return delta_lat, delta_lon

def spherical_distance(s1, s2):
    """Find distance between between s1 and s2 on geodesic of sphere

    Assume s1 is a scalar spherical coordinate (r, lat, lon)
    and s2 can be a vector array n*3


    Dist will be the same size as s2
    """
    r1, lat1, lon1 = s1[np.newaxis, 0], s1[np.newaxis, 1], s1[np.newaxis, 2]
    r2, lat2, lon2 = s2[:, 0], s2[:, 1], s2[:, 2]
    
    delta_lon = np.absolute(lon1 - lon2)

    num = np.sqrt((np.cos(lat2)*np.sin(delta_lon))**2 + (np.cos(lat1)*np.sin(lat2) - np.sin(lat1)*np.cos(lat2)*np.cos(delta_lon))**2)
    den = np.sin(lat1)*np.sin(lat2) + np.cos(lat1)*np.cos(lat2)*np.cos(delta_lon)

    delta_sigma = np.arctan2(num, den)

    # delta_sigma_cos = np.arccos(np.sin(lat1)*np.sin(lat2) + np.cos(lat1)*np.cos(lat2)*np.cos(delta_lon))

    dist = 1 * delta_sigma
    return delta_sigma
   
def radius_scale_factor(dist_sigma, std=1):
    """Scale factor based on half normal distribution

    scale - higher numbers makes more spiky area
    """
    scale = np.sqrt(2)/std/np.sqrt(np.pi) * np.exp(- dist_sigma**2 / 2 / std**2)
    return scale
