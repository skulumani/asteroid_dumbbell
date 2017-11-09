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
from mayavi import mlab
from vtk.util import numpy_support
import pdb
from point_cloud import wavefront
import utilities
# TODO: Create better function names

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

# TODO: Create a function to do surface reconstruction from a set of V
def reconstruct_numpy(verts):
    """Surface Reconstruction using VTK
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
    surf.SetNeighborhoodSize(20) # a low value makes a craggly surface
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
    v_new, f_new = vtk_poly_to_numpy(poly)

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
    v, f = vtk_poly_to_numpy(polyData)
    return v, f

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
    polyhedron = numpy_to_vtk_poly(vertices, faces)

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
    dec_vertices, dec_faces = vtk_poly_to_numpy(decimatedPoly)

    # return faces/vertices
    return dec_vertices, dec_faces

# TODO: Add documentation
def draw_polyhedron_vtk(vertices, faces):
    """Use VTK to draw a polyhedron
    """
    # create a polydata object
    polyData = numpy_to_vtk_poly(vertices, faces)

    # render and view
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polyData)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # now render
    camera = vtk.vtkCamera()
    camera.SetPosition(1, 1, 1)
    camera.SetFocalPoint(0, 0, 0)

    renderer = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(renderer)

    iren = vtk.vtkXRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    renderer.AddActor(actor)
    renderer.SetActiveCamera(camera)
    renderer.ResetCamera()
    renderer.SetBackground(0, 0, 0)

    renWin.SetSize(800,800)

    renWin.Render()
    iren.Start()

# TODO: Add documentation
def draw_polyhedron_mayavi(vertices, faces, fig):
    """Draw a polyhedron using Mayavi
    """
    x = vertices[:, 0]
    y = vertices[:, 1]
    z = vertices[:, 2]
    scalars = np.tile(0.5, x.shape)
    mesh = mlab.triangular_mesh(x, y, z, faces, color=(0.5, 0.5, 0.5), figure=fig,
                                representation='surface')

    return mesh

# TODO: Add some documentation
# TODO: Add unit testing
def polyhedron_parameters(V, F):
    """Compute some edge/face vectors for a polyhedron
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

    # TODO: Instead of searching on edges search these arrays instead
    e1_vertex_map = np.vstack((Fb, Fa)).T
    e2_vertex_map = np.vstack((Fc, Fb)).T
    e3_vertex_map = np.vstack((Fa, Fc)).T

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
    
    return (Fa, Fb, Fc, V1, V2, V3, e1, e2, e3,
            e1_vertex_map, e2_vertex_map, e3_vertex_map, 
            normal_face, e1_normal, e2_normal,e3_normal, center_face)

# TODO: Add documentation and modify inputs to search over e vertex maps instead
# TODO: THis function will give false positives for parallel edges (should be rare in a real object hopefully)
def search_edge(e1, e2, e3):

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

# TODO: Add documentation
def vertex_map_search(a_map, b_map):
    """Search and define mapping for these two sets of edge vertex maps

    """
    invalid = -1
    num_e = a_map.shape[0]
    a = 0
    b = 1

    index_map = np.full(num_e, invalid, dtype='int')
    inda1, indb1 = utilities.search_index(a_map[:, a], b_map[:, b])
    # inda2, indb2 = utilities.search_index(e1_vertex_map[inda1, b], e1_vertex_map[indb1, a])

    # check for opposite match and if so note it
    for ii in range(len(inda1)):
        if a_map[inda1[ii], b] == b_map[indb1[ii], a]:
            index_map[inda1[ii]] = indb1[ii]

    return index_map

# TODO: Remove the loops
# TODO: Don't have to repeat allt he searches (e1 in e2 is same as e2 in e1)
# TODO: Code reuse to ease all of this nonsense
def search_edge_vertex_map(e1_vertex_map, e2_vertex_map, e3_vertex_map):
    invalid = -1
    num_e = e1_vertex_map.shape[0]
    a = 0
    b = 1

    ######################## e1 searching #####################################
    e1_ind1b = vertex_map_search(e1_vertex_map, e1_vertex_map)
    e1_ind2b = vertex_map_search(e1_vertex_map, e2_vertex_map)
    e1_ind3b = vertex_map_search(e1_vertex_map, e3_vertex_map)

    
    ############################ e2 searching #################################
    e2_ind1b = vertex_map_search(e2_vertex_map, e1_vertex_map)
    e2_ind2b = vertex_map_search(e2_vertex_map, e2_vertex_map)
    e2_ind3b = vertex_map_search(e2_vertex_map, e3_vertex_map)

    ############################ e3 searching #################################
    e3_ind1b = vertex_map_search(e3_vertex_map, e1_vertex_map)
    e3_ind2b = vertex_map_search(e3_vertex_map, e2_vertex_map)
    e3_ind3b = vertex_map_search(e3_vertex_map, e3_vertex_map)
    
    return (e1_ind1b, e1_ind2b, e1_ind3b,
            e2_ind1b, e2_ind2b, e2_ind3b,
            e3_ind1b, e3_ind2b, e3_ind3b)
