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
import pdb

import numpy as np
import vtk
from vtk.util import numpy_support

import utilities

logger = logging.getLogger(__name__)

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

def ellipsoid_mesh(a, b, c, density=20):
    r"""Ellipsoid Mesh model

    verts, faces = ellipsoid_mesh(a, b, c, density)

    Parameters
    ----------
    a, b, c : floats
        Semi axes lengths (x, y, z ) directions respectively in kilometer
    density : int
        Density for the sperical coordinate parameterization

    Returns
    -------
    verts : (V, 3)
        Vertices of the ellipsoid
    faces : (F, 3)
        Faces of the ellipsoid

    See Also
    --------
    reconstruct_numpy : VTK surface reconstruction from vertices

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
    
    e_vertex_map, unique_index = np.unique(np.sort(np.vstack((e1_vertex_map, e2_vertex_map, e3_vertex_map)), axis=1), axis=0, return_index=True)

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
    # TODO : Make this a dictionary or a named tuple for ease of use

    return (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3,
            e1_vertex_map, e2_vertex_map, e3_vertex_map, 
            normal_face, e1_normal, e2_normal,e3_normal, center_face, e_vertex_map, unique_index)

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
    ind = np.where(dist == np.min(dist))[0]
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
    array_abs = np.absolute(array)
    index = np.where(array_abs == np.max(array_abs))
    sgn = np.sign(array[index])
    sgn[sgn ==0] = 1
    return sgn

def point2trimesh():
    """Find the distance from a point to a triangular mesh surface
    """
    # calculate the normal to each face
    
    # linear loop over each point
    
    # fucntion find distance to every vertex
    
    # function distance to edges

    # function distance to surfaces

    pass

def distance_to_vertices(pt, v, f, normal_face):
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

    Returns
    -------
    D : float
        Signed distance from pt to the closest vertex (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest vertices. extracted from v
    F : numpy array (m, )
        Indices of all the faces associated with the vertex P
    V : int
        Index of the closest points in v. So P = v[V,:]. There may be many 
        points

    See Also
    --------
    sign_of_largest : Finds the sign of largest element in array

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
    P = v[ind, :]
    V = ind
    
    # determine the faces that are associated with any of the vertices in ind
    F = np.where(np.isin(ind, f) == True)[0]
    assert (len(F) >= 1), "Vertex {} is not connected to any face.".format(ind)

    # find normal associated with these faces
    N = normal_face[F, :]

    # return the signed distance (inside or outside of face)
    coeff = np.einsum('ij,ij->i',N, pt-P)
    sgn = sign_of_largest(coeff)
    D = dist * sgn
    
    # TODO Better variables names
    return D, P, F, V

def distance_to_edges(pt, V, F, normal_face, edge_vertex_map,
                      edge_face_map):
    r"""Compute distance to all edges and output minimum

    D, P, F, V = distance_to_edges(pt, V, F, normal_face,
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
        Each element of tuple defines the mapping between the vertices and edges.
        The size in each are (# faces, 4). The first column defines the face number.
        The last three columns give you the matching edge and the associated face.
        Columns show, e1, e2, e3 and rows are the face numbers.

    Returns
    -------
    D : float
        Signed distance from pt to the closest edge (+ outside, - inside)
    P : numpy array (3, )
        Location of the closest point. This will lie on the closest edge
    F : numpy array (m, )
        Indices of all the faces associated with the edge (in V)
    V : int
        This defines the closest edge. e = v[V[0],:] - v[V[1], :] It holds
        the element numbers in the list of vertices which make up the edge

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------

    .. [1] OROURKE, Joseph. Computational Geometry in C. 2 ed. Cambridge
    University Press, 1998.
    """
    num_v = V.shape[0]
    num_f = F.shape[0]
    num_e = 3 * (num_v - 2)
    # put all the edges in an array

    # calculate all the edges - zero indexing for python
    Fa = F[:, 0] 
    Fb = F[:, 1]
    Fc = F[:, 2]

    e1_vertex_map, e2_vertex_map, e3_vertex_map = edge_vertex_map
    e1_face_map, e2_face_map, e3_face_map = edge_face_map

    edges = np.concatenate((e1_vertex_map, e2_vertex_map, e3_vertex_map))
    
    # find the parameteric intersection parameter for every edge (t)
    v1 = V[edges[:, 1], :]
    v2 = V[edges[:, 0], :]
    
    a = v1 - pt
    b = v2 - v1

    edge_param = - np.einsum('ij,ij->i', a, b) / np.linalg.norm(b, axis=1)**2
    
    # exclude intersections that are outside of the range 0, 1
    edge_param[edge_param <= 0] = np.nan

    mask = ~np.isnan(edge_param)
    mask[mask] &= edge_param[mask] >= 1
    edge_param[mask] = np.nan

    # find the distance between intersection (on edge) and point
    edge_intersections = v1 +edge_param[:, np.newaxis] * b
    edge_dist = pt - edge_intersections
    dist = np.sqrt(np.einsum('ij,ij->i', edge_dist, edge_dist))
    
    # find minimum distance and edge index and intersection piont
    ind = np.nanargmin(dist)
    D = dist[ind] # minimum distance to the edge
    P = edge_intersections[ind,:] # point on edge that is closest
    
    # find out which edge (e1, e2, e3) for ind (the minimum)
    edge_face_map_row = edge_face_map[ind // num_f][ind % num_f, :]
    F = edge_face_map_row[edge_face_map_row != -1] # faces associated with minimum edge
    
    N = normal_face[F, :]

    # return the signed distance (inside or outside of face)
    coefficients = np.dot(N, pt-P)
    sgn = sign_of_largest(coefficients)
    D = D * sgn 

    # determine the faces and the edge associated with this intersection
    V = edges[ind, :] # vertices that define this minimum edge
    # TODO Better variables names
    return D, P, F, V

def distance_to_faces(pt, V, F, normal_face):
    r"""Find distance to every face and output the closest one

    D, P, F, V = distance_to_faces(pt, V, F, normal_face)

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
    num_v = V.shape[0]
    num_f = F.shape[0]
    num_e = 3 * (num_v - 2)

    # extract out all the  vertices
    Fa, Fb, Fc = F[:, 0], F[:, 1], F[:, 2]
    v1, v2, v3 = V[Fa, :], V[Fb, :], V[Fc, :]

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
    
    np.testing.assert_allclose(alpha + beta + gamma, 1, err_msg='Barycentric coordinates are not valid') 

    barycentric = np.stack((alpha, beta, gamma), axis=1)
    # exclude intersections that don't lie inside the faces
    dist[np.absolute(dist) <= np.finfo(float).eps] = np.nan
    dist[np.any(barycentric < 0, axis=1)] = np.nan
    dist[np.any(barycentric >= 1, axis=1)] = np.nan

    # determine the closest face
    # TODO: Better variable names
    try:
        ind = np.nanargmin(np.absolute(dist))
        D = dist[ind]
        P = surf_intersections[ind, :]
        V = V[F[ind,:], :]
        F = ind

    except ValueError as err:
        logger.warn('The point {} is not in view of any face: {}'.format(pt, err))

        D = []
        P = []
        V = []
        F = []

    return D, P, F, V

