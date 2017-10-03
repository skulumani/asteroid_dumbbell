"""Testing out vtk

"""

import vtk
import numpy as np
from vtk.util.colors import tomato

def vtk_example():
    # generate a polygon data for a cube
    cube = vtk.vtkCubeSource()

    # create a mapper for the cube data
    cube_mapper = vtk.vtkPolyDataMapper()
    cube_mapper.SetInputData(cube.GetOutput())

    # connect the mapper to an actor
    cube_actor = vtk.vtkActor()
    cube_actor.SetMapper(cube_mapper)
    cube_actor.GetProperty().SetColor(1.0, 0.0, 0.0)

    # render the cube actor
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.0, 0.0, 0.0)
    renderer.AddActor(cube_actor)

    # create a render window
    render_window = vtk.vtkRenderWindow()
    render_window.SetWindowName("Simple VTK Scene")
    render_window.SetSize(400, 400)
    render_window.AddRenderer(renderer)

    # create the interactor
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(render_window)

    # initialize the interactor
    interactor.Initialize()
    render_window.Render()
    interactor.Start()

def vtk_cylinder():
    # create a polygon cylinder
    cylinder = vtk.vtkCylinderSource()
    cylinder.SetResolution(8)

    # the mapper pushes the geometry into the graphics library
    cylinderMapper = vtk.vtkPolyDataMapper()
    cylinderMapper.SetInputConnection(cylinder.GetOutputPort())

    # the actor is a grouping mechanics
    cylinderActor = vtk.vtkActor()
    cylinderActor.SetMapper(cylinderMapper)
    cylinderActor.GetProperty().SetColor(tomato)
    cylinderActor.RotateX(30.0)
    cylinderActor.RotateY(-45.0)

    # create the graphics structure.
    # create the render window 
    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    # Add the actors tot he renderer, set background and size
    ren.AddActor(cylinderActor)
    ren.SetBackground(0.1, 0.2, 0.4)
    renWin.SetSize(200, 200)

    # initialize the interactor
    iren.Initialize()

    # zoom in the window
    ren.ResetCamera()
    ren.GetActiveCamera().Zoom(1.5)
    renWin.Render()

    # start the event loop
    iren.Start()

def vtk_distance():
    p0 = (0, 0, 0)
    p1 = (1, 1, 1)

    distSquared = vtk.vtkMath.Distance2BetweenPoints(p0, p1)
    dist = np.sqrt(distSquared)

    print("p0 = {}".format(p0))
    print("p1 = {}".format(p1))
    print("distance squared = {}".format(distSquared))
    print("distance = {}".format(dist))

def vtk_plywriter():
    """Write to ply writer
    """


if __name__ == '__main__':
    vtk_cylinder()
    vtk_distance()
