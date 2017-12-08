import vtk
import numpy as np
from vtk.util import numpy_support
from point_cloud import wavefront
import itertools, pdb
import logging
logger = logging.getLogger(__name__)

class RayCaster(object):
    """Ray casting object
    """

    def __init__(self, polydata, flag='bsp'):
        """Initialize the ray caster

        polydata - vtkPolyData object holding the mesh
        caster - raycasting object holding vtkOBBtree
        """

        self.polydata = polydata
        self.caster = None
        self.flag = flag

        # create the caster VTK object
        self.__initCaster()

    def __initCaster(self):
        """Initialize a raycaster

        Internal method that just creates a vtkOBBTree object and setup
        """
        if self.flag == 'bsp':
            self.caster = vtk.vtkModifiedBSPTree()
        elif self.flag == 'obb':
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
    def loadmesh(v, f, scale=1.0, flag='bsp'):
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
        rT = RayCaster(polydata, flag=flag)

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

    
    def castray(self, psource, ptarget, all_out=False):
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
        all_out : bool
            Output all intersections or just the first one

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
        tol = 1e-9

        # perform the actual raycasting
        if self.flag == 'bsp':
            code = self.caster.IntersectWithLine(psource, ptarget,
                                                tol,
                                                pointsVTKintersection, None)
        elif self.flag == 'obb':
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
        
        if intersection.size > 3 and not all_out:
            return intersection[0, :]
        else:
            return intersection
    
    # TODO Add documentation and unit testing
    def castarray(self, ps, targets):
        """Only append the minimum distance intersection
        """
        # TODO Debug the array creation
        all_intersections = []
        for ii, pt in enumerate(targets):
            intersection = self.castray(ps, pt)
            all_intersections.append(intersection)

        return np.array(all_intersections)

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

    def castsensor(self, state, sensor, max_depth=50):
        """Return depth measurements from a simulated lidar

        state should be in the asteroid fixed frame
        R is the transformation from the dumbbell body frame to the asteroid frame

        return all of the measurements from the FOV of sensor

        """

        # extract out current state of satellite (position and attitude)
        
        # take the sensor and get all of the unit vectors associated with senso

        # determine the targets (sensor * distance)

        # do the point intersection using self (caster)

        pass

# TODO Test out rotating and plotting that as well
class Lidar(object):
    r"""LIDAR object

    Create an object with unit vectors defining the measurement vectors

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu
    """ 

    def __init__(self, view_axis=np.array([1, 0, 0]),
                 up_axis=np.array([0, 0, 1]),
                 fov=np.deg2rad(np.array([7, 7])), sigma=0.2,
                 dist=1, num_step=3):
        """Initialize the object

        Parameters
        ----------
        view_axis : (3,) array
            View axis of the sensor
        up_axis : (3,) array
            Up vector for the sensor (in the camera frame)
        fov : (2) array
            horizontal and vertical field of view in  radians
        sigma : float
            3sigma uncertainty in meters
        dist : float
            Distance to scale each unit vector
        num_step : int
            number of steps across the field of view to define the arrays

        """
        

        right_axis = np.cross(view_axis, up_axis)

        H = np.tan(fov[1]/2) * dist
        W = np.tan(fov[0]/2) * dist
        
        # center of view fustrum
        c = view_axis * dist
        
        # corners of the fustrum
        hsteps = np.linspace(-H, H, num_step)
        wsteps = np.linspace(-W, W, num_step)
        
        self.lidar_arr = np.zeros((num_step**2, 3))
        # define all the unit vectors for the sensor
        for ii, (h, w) in enumerate(itertools.product(hsteps, wsteps)):
            lidar_vec = c + h * up_axis + w * right_axis
            self.lidar_arr[ii, :] = lidar_vec / np.linalg.norm(lidar_vec)

        # save things to object
        self.up_axis = up_axis
        self.view_axis = view_axis
        self.right_axis = right_axis

    def rotate_fov(self, R_body2frame):
        """Rotate the entire FOV by a given rotation matrix

        This method will rotate the lidar arr by the given rotation matrix

        Only outputs the unit vectors. User can find a vector by multiplying by a distance


        """
        # initialize an object

        # rotate all the vector by the new rotation matrix
        self.lidar_arr = R_body2frame.dot(self.lidar_arr.T).T 

        return self.lidar_arr
