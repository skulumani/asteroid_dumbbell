import numpy as np
import itertools, pdb

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

    
