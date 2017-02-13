# module for rotation kinematics
import numpy as np

def rot1(angle):
    """
    Elementary rotation about the first axis. For column vectors b = R*a
    """
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    
    rot_mat = np.identity(3)
    rot_mat[1,1] = cos_a
    rot_mat[1,2] = sin_a
    rot_mat[2,1] = -sin_a
    rot_mat[2,2] = cos_a
    
    return rot_mat
    
def rot2(angle):
    """
    Elementary rotation about the second axis. For column vectors b = R a
    """
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    
    rot_mat = np.identity(3)
    rot_mat[0,0] = cos_a
    rot_mat[0,2] = -sin_a
    rot_mat[2,0] = sin_a
    rot_mat[2,2] = cos_a
    
    return rot_mat
    
def rot3(angle):
    """
    Elementary rotation about the third axis. For column vectors b = R a
    """
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    
    rot_mat = np.identity(3)
    rot_mat[0,0] = cos_a
    rot_mat[0,1] = sin_a
    rot_mat[1,0] = -sin_a
    rot_mat[1,1] = cos_a
    
    return rot_mat
    
def hat_map(vec):
    """Return that hat map of a vector
    
    Inputs: 
        vec - 3 element vector

    Outputs:
        skew - 3,3 skew symmetric matrix

    """

    skew = np.array(
                    [0, -vec[2], vec[1]],
                    [vec[2], 0, -vec[0]],
                    [-vec[1], vec[0], 0])

    return skew

if __name__ == "__main__":
    angle = math.pi/4.0
    
    print(rot1(angle))