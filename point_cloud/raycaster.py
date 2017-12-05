import vtk
import numpy as np
from vtk.util import numpy_support
from point_cloud import wavefront
import logging

logger = logging.getLogger(__name__)

class RayCaster(object):
    """Ray casting object
    """

    def __init__(self, polydata):
        """Initialize the ray caster

        polydata - vtkPolyData object holding the mesh
        caster - raycasting object holding vtkOBBtree
        """

        self.polydata = polydata
        self.caster = None
        
        # create the caster VTK object
        self.__initCaster()

    def __initCaster(self):
        """Initialize a raycaster

        Internal method that just creates a vtkOBBTree object and setup
        """
        self.caster = vtk.vtkOBBTree()
        # set the object polydata as the dataset
        self.caster.SetDataSet(self.polydata)
        self.caster.BuildLocator()
    
    def __updateCaster(self):
        """Update the vtkOBBTree if the polydata changes
        """
        self.caster.SetDataSet(self.polydata)
        self.caster.BuildLocator()

    @staticmethod
    def loadmesh(v, f, scale=1.0):
        """Create a vtkPolydata from a set of vertices and faces

        v : vertices
        f : faces
        scale : scale the units - factor to multiply by
        """
        polydata = wavefront.meshtopolydata(v, f)

        # error check the number of points in the mesh
        if polydata.GetNumberOfPoints() != v.shape[0]:
            raise ValueError("Polydata does not match")
            return None

        # initialize a raycaster
        rT = RayCaster(polydata)

        # scale the polydata if nothing is given
        if scale != 1.0:
            rT.scalemesh(scale)

        return rT

    def scalemesh(self, scale):
        """Scales (multiplies) the mesh by scale

        Simply multiplies all the vertices by the given scale factor

        scale : float
            Apply to the polydata

        """
        transform = vtk.vtkTransform()
        transformFilter = vtk.vtkTransformPolyDataFilter()
        
        transform.Scale(scale, scale, scale)

        transformFilter.SetInputData(self.polydata)

        transformFilter.SetTransform(transform)
        transformFilter.Update()

        self.polydata = transformFilter.GetOutput()

        # update the raycaster
        self.__updateCaster()

    
    def castray(self, psource, ptarget):
        """Perform ray-casting for a given ray
        
        intersection = caster.castray(source, target)

        This method will perform raycasting for a ray going from the source
        to the target. It returns a numpy array which defines the intersections
        of the ray with the polydata mesh within this object.

        Parameters
        ----------
        psource : numpy array (3,)
            Start of the ray (usually the position of the satellite)
        ptarget : numpy array (3,)
            Target of the ray ( center of the asteroid or elsewhere)
        
        Returns
        -------
        intersections : numpy array (n, 3)
            All of the intersections of the ray with the mesh
        
        Author
        ------
        Shankar Kulumani		GWU		skulumani@gwu.edu
        """ 
        
        # create a vtkPoints object to store all the intersections
        pointsVTKintersection = vtk.vtkPoints()

        # perform the actual raycasting
        code = self.caster.IntersectWithLine(psource, ptarget,
                                             pointsVTKintersection, None)

        # error checking of the code
        if code == 0:
            logger.info(
                "No intersection points for 'source': " + str(psource) 
                + " and 'target': " + str(ptarget))
            return np.array([])

        elif code == -1: # source is inside of the surface
            logger.info(
                "The point 'source': " + str(psource) + " is inside mesh.")

        # extract intersections for vtkPoints
        intersection = numpy_support.vtk_to_numpy(pointsVTKintersection.GetData())

        return intersection

    def ispointinside(self, point):
        """Check if a point lies inside the mesh using vtk
        
        bool = caster.ispointinside(point)

        Parameters
        ----------
        point : (3,) numpy array
            corrdinates of the point to check

        Returns
        -------
        bool : True or False boolean
            True if inside and False if outside

        Author
        ------
        Shankar Kulumani		GWU		skulumani@gwu.edu
        """
        code = self.caster.InsideOrOutside(point)

        if code == -1:
            return True
        else:
            return False

    def distance(self, pa, pb):
        """Calculate the distance between two points
        """

        distance = np.linalg.norm(pa - pb)
        return distance


