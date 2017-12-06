import numpy as np
import itertools, pdb

class Lidar(object):
    """LIDAR object
    
    Hold the pointing direction of the camera in body frame

    Provide methods to derive all the unit vectors for the LIDAR measurements

    Another method to transform into another frame given a transformation matrix

    """

    def __init__(self, view_axis=np.array([1, 0, 0]),
                 up_axis=np.array([0, 0, 1]),
                 fov=np.deg2rad(np.array([7, 7])), sigma=0.2,
                 dist=1, num_step=3):
        """Initialize the object

        cam_axis : define the pointing direction of center of LIDAR in the
        body fixed frame

        fov : horizontal and vertical field of view in radians

        sigma : 3 sigma uncertainty in depth measurements
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
            lidar_arr[ii, :] = lidar_vec / np.linalg.norm(lidar_vec)

        # save things to object
        self.lidar_arr = lidar_arr
        self.up_axis = up_axis
        self.view_axis = view_axis
        self.right_axis = right_axis

    def rotate_fov(self, R_body2inertial):
        """Rotate the entire FOV by a given rotation matrix
        """

        # rotate all the vector by the new rotation matrix
    


