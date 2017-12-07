"""Visualize using VTK or Mayavi for fancy 3D graphics and things

Use VTK/Mayavi to make fancy plots

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
import vtk
from vtk.util import numpy_support
from mayavi import mlab
import numpy as np


def draw_polyhedron_vtk(vertices, faces):
    r"""Plot a polyhedron using VTK

    mesh = draw_polyhedron_vtk(v, f)

    Parameters
    ----------
    vertices : (v, 3) numpy array
        Array holding all the vertices of the mesh
    faces : (f, 3) numpy array
        Array holding all of the faces of the mesh

    Returns
    -------
    none

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    # create a polydata object
    polyData = meshtopolydata(vertices, faces)

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

def draw_polyhedron_mayavi(vertices, faces, fig):
    r"""Plot a polyhedron using Mayavi

    mesh = draw_polyhedron_mayavi(v, f, fig)

    Parameters
    ----------
    vertices : (v, 3) numpy array
        Array holding all the vertices of the mesh
    faces : (f, 3) numpy array
        Array holding all of the faces of the mesh
    fig : mlab.figure()
        Figure to plot the mesh into

    Returns
    -------
    mesh : mlab.triangular_mesh
        Mesh object from mayavi

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    x = vertices[:, 0]
    y = vertices[:, 1]
    z = vertices[:, 2]
    scalars = np.tile(0.5, x.shape)
    mesh = mlab.triangular_mesh(x, y, z, faces, color=(0.5, 0.5, 0.5), figure=fig,
                                representation='surface')

    return mesh

def vtk_addPoint(renderer, p, radius=0.1, color=[0.0, 0.0, 1]):
    r"""Add a point to a VTK render

    vtk_addPoint(renderer, point, radius=0.1, color=[R, G, B])

    Parameters
    ----------
    renderer : vtk.vtkRenderer()
        Renderer object in vtk
    p : (3,) array/tuple
        Point to plot in VTK
    radius : float
        Size of the point ( graphing units)
    color : (3,) array
        RGB color of point

    Returns
    -------
    none

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    res = 100

    point = vtk.vtkSphereSource()
    point.SetCenter(p)
    point.SetRadius(radius)
    point.SetPhiResolution(res)
    point.SetThetaResolution(res)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(point.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(color)

    renderer.AddActor(actor)
    
    return 0

def vtk_addLine(renderer, p1, p2, color=[0.0, 0.0, 1.0]):
    r"""Add a line to a VTK render

    vtk_addLine(renderer, p1, p2, radius=0.1, color=[R, G, B])

    Parameters
    ----------
    renderer : vtk.vtkRenderer()
        Renderer object in vtk
    p1 : (3,) array/tuple
        Start Point to plot in VTK
    p2 : (3,) array/tuple
        End Point to plot in VTK
    radius : float
        Size of the point ( graphing units)
    color : (3,) array
        RGB color of point

    Returns
    -------
    none

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    line = vtk.vtkLineSource()
    line.SetPoint1(p1)
    line.SetPoint2(p2)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(line.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(color)

    renderer.AddActor(actor)
    
    return 0

def vtk_addPoly(renderer, polydata, color=[0.0, 0.0, 1.0]):
    r"""Add a polyhedron to a VTK render

    vtk_addPoly(renderer, polydata, color=[R, G, B])

    Parameters
    ----------
    renderer : vtk.vtkRenderer()
        Renderer object in vtk
    polydata : vtk.vtkPolyData
        Polyhedron object ( can convert numpy arrays to mesh from wavefront)
    color : (3,) array
        RGB color of point

    Returns
    -------
    none

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    # now viusalize
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    mapper.SetScalarRange(0, 7)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer.AddActor(actor)

    return 0

def vtk_show(renderer, width=800, height=600):
    r"""Start a vtk renderer window and interact

    vtk_show(renderer, width, height)

    Parameters
    ----------
    renderer : vtk.vtkRenderer()
        Renderer object in vtk
    width, height : int
        Width and height in pixels of resulting window

    Returns
    -------
    none

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    # now render
    camera = vtk.vtkCamera()
    camera.SetPosition(2, 2, 2)
    camera.SetFocalPoint(0, 0, 0)

    renderer.SetActiveCamera(camera)
    renderer.ResetCamera()
    renderer.SetBackground(0, 0, 0)

    renderWindow = vtk.vtkRenderWindow()
    # renderWindow.SetOffScreenRendering(1)
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(width, height)
    
    iren = vtk.vtkXRenderWindowInteractor()
    iren.SetRenderWindow(renderWindow)

    renderWindow.Render()
    iren.Start()

def vtk_renderer():
    """Just creates a renderer so you don't have to import VTK everywhere
    """
    return vtk.vtkRenderer()

# TODO: Add Mayvi polyhedron, line, and point helper functions
