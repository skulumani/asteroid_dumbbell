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

def reduced_mesh_generation(filename='./data/itokawa_low.obj'):
    """Read in the OBJ shape file, degrade the number of vectors and then 
    create a surface
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
