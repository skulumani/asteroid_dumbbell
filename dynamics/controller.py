from kinematics import attitude
import numpy as np
import scipy.linalg
import pdb

def attitude_controller(time, state, ext_moment, dum):
    r"""Geometric attitude controller on SO(3)

    This function will determine an attitude control input for a rigid spacecraft around an asteroid.
    The function is setup to work for a vehicle defined in the inertial frame relative to an asteroid.

    Parameters
    ----------
    self : dumbbell instance
        Instance of dumbbell class with all of it's parameters
    time : float
        Current time for simulation which is used in the desired attitude trajectory
    state : array_like (18,)
        numpy array defining the state of the dumbbell
        position - position of the center of mass wrt to the inertial frame
        and defined in the inertial frame (3,)
        velocity - velocity of the center of mass wrt to teh inertial frame
        and defined in the inertial frame (3,)
        R_b2i - rotation matrix which transforms vectors from the body
        frame to the inertial frame (9,)
        angular_velocity - angular velocity of the body frame with respect
        to the inertial frame and defined in the body frame (3,)
    ext_moment : array_like (3,)
        External moment in the body fixed frame

    Returns
    -------
    u_m : array_like (3,)
        Body fixed control moment

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------
    
    .. [1] LEE, Taeyoung, LEOK, Melvin y MCCLAMROCH, N Harris. "Control of
    Complex Maneuvers for a Quadrotor UAV Using Geometric Methods on Se
    (3)". arXiv preprint arXiv:1003.2005. 2010, 

    Examples
    --------

    """ 
    # extract the state
    pos = state[0:3] # location of the center of mass in the inertial frame
    vel = state[3:6] # vel of com in inertial frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame
    # compute the desired attitude command
    Rd, Rd_dot, ang_vel_d, ang_vel_d_dot = desired_attitude(time)
    # determine error between command and current state
    eR = 1/2 * attitude.vee_map(Rd.T.dot(R) - R.T.dot(Rd))
    eW = ang_vel - R.T.dot(Rd).dot(ang_vel_d)
    # compute attitude input
    u_m = (-dum.kR*eR -dum.kW*eW + np.cross(ang_vel,dum.J.dot(ang_vel)) 
            -dum.J.dot( attitude.hat_map(ang_vel).dot(R.T).dot(Rd).dot(ang_vel_d)-
                R.T.dot(Rd).dot(ang_vel_d_dot)) - ext_moment)
    return u_m

def translation_controller(time, state, ext_force, dum):
    """SE(3) Translational Controller

    Inputs:

    Outputs:
        u_f - force command in the dumbbell frame

    """

    # extract the state
    pos = state[0:3] # location of the center of mass in the inertial frame
    vel = state[3:6] # vel of com in inertial frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame
    
    m = dum.m1 + dum.m2

    # figure out the desired trajectory
    x_des, xd_des, xdd_des = desired_translation(time)

    # compute the error
    ex = pos - x_des
    ev = vel - xd_des
    # compute the control
    u_f = -dum.kx * ex -dum.kv * ev - ext_force + m * xdd_des

    return u_f

def desired_attitude(time, alpha=2*np.pi/100, axis=np.array([0, 1, 0])):
    """Desired attitude trajectory

    This function will output a desired attitude trajectory. The controller will use this 
    trajectory in it's computations. The outputs will be the desired attitude matrix and
    the desired angular velocity:

    Outputs:
        Rd_sc2int - 3x3 array defining the transformation from the spacecraft frame to the
            inertial frame
        w_sc2int - 3 array defining the angular velocity of the spacecraft frame with respect
            to the inertial frame and defined in the spacecraft fixed frame

    """
    Rd = scipy.linalg.expm(alpha * time * attitude.hat_map(axis) ) 
    Rd_dot = alpha * attitude.hat_map(axis).dot(
                scipy.linalg.expm(alpha * time * attitude.hat_map(axis)))

    ang_vel_d = attitude.vee_map(Rd.T.dot(Rd_dot))
    ang_vel_d_dot = np.zeros_like(ang_vel_d)

    return (Rd, Rd_dot, ang_vel_d, ang_vel_d_dot) 

def desired_translation(time, alpha=2*np.pi/100):
    """Desired translational trajectory

    This function will output the desired translational states, namely the desired position and
    velocity. This position and velocity will be defined in the inertial reference frame.

    """
    x_des = np.array([1.5, 0.2*np.cos(alpha * time), 0.5*np.sin(alpha * time)])
    xd_des = np.array([0, - alpha * 0.2 * np.sin(alpha * time), alpha * 0.5 * np.cos(alpha * time)])
    xdd_des = np.array([0, - alpha**2 * 0.2 * np.cos(alpha * time), - alpha**2 * 0.5 * np.sin(alpha * time)])

    return (x_des, xd_des, xdd_des)

def body_fixed_hovering_attitude(time, state):
    """Desired attitude to ensure that the x axis is always pointing at the origin (asteroid)

    """
    # extract out the states
    pos = state[0:3] # location of the center of mass in the inertial frame
    vel = state[3:6] # vel of com in inertial frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame
    # compute the desired attitude to ensure that the body fixed x axis is pointing at the origin/asteroid
    # each column of the rotation matrix is the i-th body fixed axis as represented in the inertial frame
    b1_des = - pos / np.linalg.norm(pos)
    b3_des = np.array([0, 0, 1]) - (np.array([0, 0, 1]).dot(b1_des) * b1_des)   # ensure the body z axis is always parallel with the asteroid/inertial z axis
    b3_des = b3_des / np.linalg.norm(b3_des)
    b2_des = np.cross(b3_des, b1_des)

    Rd = np.stack((b1_des, b2_des, b3_des), axis=1)

    Rd_dot = np.zeros((3,3))

    ang_vel_d = np.zeros(3) 
    ang_vel_d_dot = np.zeros(3) 

    return (Rd, Rd_dot, ang_vel_d, ang_vel_d_dot)

def body_fixed_hovering_translation(time):
    """Desired translational states for vertical landing on asteroid

    """

