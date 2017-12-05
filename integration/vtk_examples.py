"""Testing out vtk

"""

import vtk
import numpy as np
from vtk.util import numpy_support
from vtk.util.colors import tomato
from vtk.util.misc import vtkGetDataRoot
import os
import string
import time
from point_cloud import wavefront, raycaster
import pdb

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

    filename = '/tmp/writeply.ply'

    sphereSource = vtk.vtkSphereSource()
    sphereSource.Update()

    plyWriter = vtk.vtkPLYWriter()
    plyWriter.SetFileName(filename)
    plyWriter.SetInputConnection(sphereSource.GetOutputPort())
    plyWriter.Write()

    # read and display for verification
    reader = vtk.vtkPLYReader()
    reader.SetFileName(filename)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    renderer.AddActor(actor)
    renderer.SetBackground(0.3, 0.6, 0.3)

    renderWindow.Render()
    renderWindowInteractor.Start()


def vtk_stlwriter():
    filename = '/tmp/test.stl'

    sphereSource = vtk.vtkSphereSource()
    sphereSource.Update()

    # write the stl file to a disk
    stlWriter = vtk.vtkSTLWriter()
    stlWriter.SetFileName(filename)
    stlWriter.SetInputConnection(sphereSource.GetOutputPort())
    stlWriter.Write()

    # read and display for verification
    reader = vtk.vtkSTLReader()
    reader.SetFileName(filename)

    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(reader.GetOutput())
    else:
        mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # create a rendering window and renderer
    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)

    # create a renderwindowinteractor
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    # assign actor to the renderer
    ren.AddActor(actor)

    # enable user interface interactor
    iren.Initialize()
    renWin.Render()
    iren.Start()

def vtk_stlreader(filename='/tmp/test.stl'):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(filename)

    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(reader.GetOutput())
    else:
        mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    ren.AddActor(actor)
    iren.Initialize()
    renWin.Render()
    iren.Start()

def reconstruct_surface():
    """Example of constructing a surface from a point cloud

    https://github.com/Kitware/VTK/blob/a1a94d0ca96854fe72480cf2ec031a533b129b04/Examples/Modelling/Python/reconstructSurface.py
    """

    pointSource = vtk.vtkProgrammableSource()

    def readPoints():
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

    pointSource.SetExecuteMethod(readPoints)
    
    surf = vtk.vtkSurfaceReconstructionFilter()
    surf.SetInputConnection(pointSource.GetOutputPort())


    cf = vtk.vtkContourFilter()
    cf.SetInputConnection(surf.GetOutputPort())
    cf.SetValue(0, 0.0)
    
    reverse = vtk.vtkReverseSense()
    reverse.SetInputConnection(cf.GetOutputPort())
    reverse.ReverseCellsOn()
    reverse.ReverseNormalsOn()

    map = vtk.vtkPolyDataMapper()
    map.SetInputConnection(reverse.GetOutputPort())
    map.ScalarVisibilityOff()

    surfaceActor = vtk.vtkActor()
    surfaceActor.SetMapper(map)
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
   
def step1_cone():
    """Example of dealing with VTK
    https://github.com/Kitware/VTK/blob/master/Examples/Tutorial/Step1/Python/Cone.py
    """
    
    # Create a source object - returns vtkPolyData type
    cone = vtk.vtkConeSource()
    cone.SetHeight(3.0)
    cone.SetRadius(1.0)
    cone.SetResolution(10)
    
    # no filters here - connect the source to the mapper to convert the data to a graphics primitvie
    coneMapper = vtk.vtkPolyDataMapper()
    coneMapper.SetInputConnection(cone.GetOutputPort())

    # create teh actor to represent the cone - this is what handles transforming the graphic primitives into something that can be rendered
    coneActor = vtk.vtkActor()
    coneActor.SetMapper(coneMapper)

    # crete the renderer - this is the figure/viewport where we create the image
    ren1 = vtk.vtkRenderer()
    ren1.AddActor(coneActor)
    ren1.SetBackground(0.1, 0.2, 0.4)

    # create the render window - this is what shows up on the screen
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren1)
    renWin.SetSize(300, 300)

    # now animate
    for i in range(0, 360):
        time.sleep(0.03)

        renWin.Render()
        ren1.GetActiveCamera().Azimuth(1)

def step2_observer():
    """
    This is an example of using a callback via an observer on the renderer

    Everytime it's called a function is run
    """
    def myCallback(obj, string):
        print("Starting a render")

    # create a basic pipeline
    cone = vtk.vtkConeSource()
    cone.SetHeight(3.0)
    cone.SetRadius(1.0)
    cone.SetResolution(10)

    coneMapper = vtk.vtkPolyDataMapper()
    coneMapper.SetInputConnection(cone.GetOutputPort())
    coneActor = vtk.vtkActor()
    coneActor.SetMapper(coneMapper)

    ren1 = vtk.vtkRenderer()
    ren1.AddActor(coneActor)
    ren1.SetBackground(0.1, 0.2, 0.4)

    # add observer
    ren1.AddObserver("StartEvent", myCallback)
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren1)
    renWin.SetSize(300, 300)

    for i in range(0, 360):
        time.sleep(0.03)
        renWin.Render()
        ren1.GetActiveCamera().Azimuth(1)

def step3_multiple_renderer():
    """Create multiple renderers inside a render window
    """

    cone = vtk.vtkConeSource()
    cone.SetHeight(3.0)
    cone.SetRadius(1.0)
    cone.SetResolution(100)
    
    coneMapper = vtk.vtkPolyDataMapper()
    coneMapper.SetInputConnection(cone.GetOutputPort())

    coneActor = vtk.vtkActor()
    coneActor.SetMapper(coneMapper)

    ren1 = vtk.vtkRenderer()
    ren1.AddActor(coneActor)
    ren1.SetBackground(0.1, 0.2, 0.4)
    ren1.SetViewport(0.0, 0.0, 0.5, 1.0)

    ren2 = vtk.vtkRenderer()
    ren2.AddActor(coneActor)
    ren2.SetBackground(0.1, 0.2, 0.4)
    ren2.SetViewport(0.5, 0.0, 1.0, 1.0)

    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren1)
    renWin.AddRenderer(ren2)
    renWin.SetSize(600, 300)

    ren1.ResetCamera()
    ren1.GetActiveCamera().Azimuth(90)

    for i in range(0, 360):
        time.sleep(0.03)
        renWin.Render()
        ren1.GetActiveCamera().Azimuth(1)
        ren2.GetActiveCamera().Azimuth(1)
    
def read_obj(filename):
    """Test to read and display a wavefront OBJ

    """
        
    reader = vtk.vtkOBJReader()
    reader.SetFileName(filename)

    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(reader.GetOutput())
    else:
        mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    ren.AddActor(actor)
    iren.Initialize()
    renWin.Render()
    iren.Start()

def write_obj(infile='./data/itokawa_low.obj', outfile='./data/point_clouds/itokawa_low_vtkwriter'):
    """Read from a OBJ and then write to a different one
    """

    reader = vtk.vtkOBJReader()
    reader.SetFileName(infile)

    # write the stl file to a disk
    objWriter = vtk.vtkOBJExporter()
    objWriter.SetFilePrefix(outfile)

    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(reader.GetOutput())
    else:
        mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    ren.AddActor(actor)
    iren.Initialize()
    renWin.Render()
    iren.Start()

    objWriter.SetRenderWindow(renWin)
    objWriter.Write()

def obj_to_numpy(filename):
    """Convert OBJ to a numpy array

    https://stackoverflow.com/questions/23138112/vtk-to-maplotlib-using-numpy
    """
    reader = vtk.vtkOBJReader()
    reader.SetFileName(filename)
    reader.Update()

    output = reader.GetOutput()
    points = output.GetPoints()
    verts = numpy_support.vtk_to_numpy(points.GetData())

    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(reader.GetOutput())
    else:
        mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    ren.AddActor(actor)
    iren.Initialize()
    renWin.Render()
    iren.Start()

    return verts

def mesh_decimate(filename):
    """Input a OBJ shape model and then reduce the number of vertices
    then output to a numpy array

    https://stackoverflow.com/questions/38197212/mesh-decimation-in-python

    https://www.vtk.org/gitweb?p=VTK.git;a=blob;f=Examples/VisualizationAlgorithms/Python/deciFran.py
    """

    reader = vtk.vtkOBJReader()
    reader.SetFileName(filename)
    reader.Update()
    
    # decimate the obj file object
    inputPoly = vtk.vtkPolyData()
    inputPoly.ShallowCopy(reader.GetOutput())
    
    print("Before decimation\n\n {} vertices \n {} faces".format(inputPoly.GetNumberOfPoints(), inputPoly.GetNumberOfPolys()))
    
    # now decimate
    decimate = vtk.vtkDecimatePro()
    decimate.SetInputData(inputPoly)
    decimate.SetTargetReduction(0.5)
    decimate.Update()
    
    pdb.set_trace()
    decimatedPoly = vtk.vtkPolyData()
    decimatedPoly.ShallowCopy(decimate.GetOutput())
    print("After decimation\n\n {} vertices \n {} faces".format(decimatedPoly.GetNumberOfPoints(),decimatedPoly.GetNumberOfPolys()))

    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(reader.GetOutput())
    else:
        mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    ren.AddActor(actor)
    iren.Initialize()
    renWin.Render()
    iren.Start()


def build_triangle():
    """Build a triangle mesh using the raw points and faces
    
    https://lorensen.github.io/VTKExamples/site/Python/PolyData/ColoredTriangle/
    """

    Points = vtk.vtkPoints()
    Triangles = vtk.vtkCellArray()

    Points.InsertNextPoint(1.0, 0.0, 0.0)
    Points.InsertNextPoint(0.0, 0.0, 0.0)
    Points.InsertNextPoint(0.0, 1.0, 0.0)

    Triangle = vtk.vtkTriangle()
    Triangle.GetPointIds().SetId(0, 0)
    Triangle.GetPointIds().SetId(1, 1)
    Triangle.GetPointIds().SetId(2, 2)
    Triangles.InsertNextCell(Triangle)

    # setup colors
    Colors = vtk.vtkUnsignedCharArray()
    Colors.SetNumberOfComponents(3)
    Colors.SetName("Colors")
    Colors.InsertNextTuple3(255, 0, 0)
    Colors.InsertNextTuple3(0, 255, 0)
    Colors.InsertNextTuple3(0, 0, 255)

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(Points)
    polydata.SetPolys(Triangles)

    polydata.GetPointData().SetScalars(Colors)
    polydata.Modified()
    if vtk.VTK_MAJOR_VERSION <= 5:
        polydata.Update()
    
    # now display it
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # now render stuff
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
    renWin.Render()
    iren.Start()


def cube():
    """Construct a cube manually

    https://www.vtk.org/Wiki/VTK/Examples/Python/DataManipulation/Cube.py
    """
    verts = [(0.0, 0.0, 0.0),
             (1.0, 0.0, 0.0),
             (1.0, 1.0, 0.0),
             (0.0, 1.0, 0.0),
             (0.0, 0.0, 1.0),
             (1.0, 0.0, 1.0),
             (1.0, 1.0, 1.0),
             (0.0, 1.0, 1.0)]
    faces = [(0, 1, 2, 3),
             (4, 5, 6, 7),
             (0, 1, 5, 4),
             (1, 2, 6, 5),
             (2, 3, 7, 6),
             (3, 0, 4, 7)]

    # cube = vtk.vtkPolyData()
    # points = vtk.vtkPoints()
    # polys = vtk.vtkCellArray() # insert cell as a number and a tuple of the vertices in the face
    # scalars = vtk.vtkFloatArray()
    
    # # load the data
    # for i in range(8):
    #     points.InsertPoint(i, verts[i])

    # for i in range(6):
    #     polys.InsertNextCell(vtk_mesh.make_vtk_idlist(faces[i]))
    
    # for i in range(8):
    #     scalars.InsertTuple1(i, i)

    # # now assign everything to the polydata object
    # cube.SetPoints(points)
    # cube.SetPolys(polys)
    # cube.GetPointData().SetScalars(scalars)
    cube = vtk_mesh.numpy_to_vtk_poly(np.array(verts), np.array(faces))

    # now viusalize
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(cube)
    mapper.SetScalarRange(0, 7)
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

    renWin.SetSize(300, 300)

    renWin.Render()
    iren.Start()

def vtk_render_polydata(poly):
    """Helper function to turn VTK poly data to a rendered window
    """
    # now viusalize
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    mapper.SetScalarRange(0, 7)
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

    renWin.SetSize(300, 300)

    renWin.Render()
    iren.Start()

def vtk_addPoint(renderer, p, radius=0.1, color=[0.0, 0.0, 1]):
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
    # now viusalize
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    mapper.SetScalarRange(0, 7)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer.AddActor(actor)

    
    return 0

def vtk_show(renderer, width=800, height=600):
    """Takes a vtkrenderer instance start the rendering window
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

def vtk_raycasting():
    """Testing out raycasting inside of VTK
    """
    
    # read and turn OBJ to polydata
    polydata = wavefront.read_obj_to_polydata('./data/shape_model/ITOKAWA/itokawa_low.obj')
    renderer = vtk.vtkRenderer() 

    pSource = np.array([-2.0, 0, 0])
    pTarget = np.array([2, 0, 0])
    
    
    # oriented bounding box for the polydata
    obbTree = vtk.vtkOBBTree()
    obbTree.SetDataSet(polydata)
    obbTree.BuildLocator()
    
    pointsVTKintersection = vtk.vtkPoints()
    code = obbTree.IntersectWithLine(pSource, pTarget, pointsVTKintersection, None)
    if code:
        points_int = numpy_support.vtk_to_numpy(pointsVTKintersection.GetData())
        for p in points_int: 
            vtk_addPoint(renderer, p , color=[0, 0, 1], radius=0.01)

    vtk_addPoly(renderer, polydata)    
    vtk_addPoint(renderer, pSource, color=[0, 1.0, 0], radius=0.01)
    vtk_addPoint(renderer, pTarget, color=[1.0, 0, 0], radius=0.01)
    vtk_addLine(renderer, pSource, pTarget)
    vtk_show(renderer)

def raycasting_visualization():
    v, f = wavefront.read_obj('./data/shape_model/ITOKAWA/itokawa_low.obj')
    caster = raycaster.RayCaster.loadmesh(v, f)
    
    renderer = vtk.vtkRenderer() 

    pSource = np.array([-2.0, 0, 0])
    pTarget = np.array([2, 0, 0])
    
    intersections = caster.castray(pSource, pTarget)
    

    for p in intersections:
        vtk_addPoint(renderer, p, color=[0, 0, 1], radius=0.01)

    vtk_addPoly(renderer, caster.polydata)
    vtk_show(renderer)

if __name__ == '__main__':
    vtk_cylinder()
    vtk_distance()
