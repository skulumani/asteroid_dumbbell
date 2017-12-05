
class Lidar(object):
    """LIDAR object
    
    Hold the pointing direction of the camera in body frame

    Provide methods to derive all the unit vectors for the LIDAR measurements

    Another method to transform into another frame given a transformation matrix

    """

    def __init__(self, cam_axis=np.array([1, 0, 0]),
                 fov=np.deg2rad(np.array([7, 7])), sigma=0.2):
        """Initialize the object

        cam_axis : define the pointing direction of center of LIDAR in the
        body fixed frame

        fov : horizontal and vertical field of view in radians

        sigma : 3 sigma uncertainty in depth measurements
        """
    pass


