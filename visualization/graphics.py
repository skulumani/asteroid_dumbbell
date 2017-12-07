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
from point_cloud import wavefront
import logging
logger = logging.getLogger(__name__)

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

def mayavi_figure():
    """Just define a figure for mayavi to plot into
    """
    return mlab.figure()

def mayavi_addPoly(fig, polydata, color=(0.5, 0.5, 0.5)):
    r"""Draw a VKT polydata object to a mayavi figure

    mesh = mayavi_addPoly(fig, polydata)

    Convert the polydata to V,F then plot inside mayavi

    Parameters
    ----------
    fig : mlab.figure()
        Mayavi figure to draw into
    polydata : vtk.vtkPolyData object
        Polyhedron object from VTK

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """
    vertices, faces = wavefront.polydatatomesh(polydata)

    scalars = np.tile(0.5, vertices.shape[0])
    mesh = mlab.triangular_mesh(vertices[:, 0], vertices[:, 1], vertices[:, 2],
                                faces, color=color, figure=fig,
                                representation='surface')

    return mesh

def mayavi_addMesh(fig, vertices, faces, color=(0.5, 0.5, 0.5)):
    r"""Draw a mesh to a mayavi figure

    mesh = mayavi_addPoly(fig, v, f, color)

    Draw mesh defined by vertices and faces to a mayavi figure

    Parameters
    ----------
    fig : mlab.figure()
        Mayavi figure to draw into
    vertices : (v, 3) numpy array
        Array holding all the vertices of the mesh
    faces : (f, 3) numpy array
        Array holding all of the faces of the mesh

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """

    scalars = np.tile(0.5, vertices.shape[0])
    mesh = mlab.triangular_mesh(vertices[:, 0], vertices[:, 1], vertices[:, 2],
                                faces, color=color, figure=fig,
                                representation='surface')

def mayavi_addLine(fig, p1, p2, color=[0, 0, 1]):
    r"""Add a line to a mayavi figure

    mayavi_addLine(fig, p1, p2, radius=0.1, color=[R, G, B])

    Parameters
    ----------
    fig : mlab.figure()
        Figure to plot into
    p1 : (3,) array/tuple
        Start Point to plot 
    p2 : (3,) array/tuple
        End Point to plot
    radius : float
        Size of the line( graphing units)
    color : (3,) array
        RGB color of line

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    line = mlab.plot3d([p1[0], p2[0]], 
                       [p1[1], p2[1]],
                       [p1[2], p2[2]],
                       color=color,
                       figure=fig,
                       tube_radius=None)
    return line

def mayavi_addPoint(fig, p, radius=0.1, color=[0, 0, 1]):
    r"""Add a point to a mayavi figure

    mayavi_addPoint(fig, p, radius=0.1, color=[R, G, B])

    Parameters
    ----------
    fig : mlab.figure()
        Figure to plot into
    p : (3,) array/tuple
        Start Point to plot 
    radius : float
        Size of the line( graphing units)
    color : (3,) array
        RGB color of line

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 
    point = mlab.plot3d(p[0], p[1], p[2], color=color, radius=radius,
                        figure=fig)
    return point
