from kinematics import attitude
import numpy as np
import scipy.linalg
import pdb

def attitude_controller(time, state, ext_moment, dum, ast, des_att_tuple):
    r"""Geometric attitude controller on SO(3)

    This function will determine an attitude control input for a rigid spacecraft around an asteroid.
    The function is setup to work for a vehicle defined in the inertial frame relative to an asteroid.

    Parameters
    ----------
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
    dum : Dumbbell Instance
        Instance of a dumbbell which defines the shape and MOI
    ast : Asteroid Instance
        Holds the asteroid model and polyhedron potential
    des_att_tuple : Desired attitude tuple
        des_att_tuple[0] - Rd desired attitude
        des_att_tuple[1] - Rd_dot desired attitude derivative
        des_att_tuple[2] - ang_vel_d angular velocity
        des_att_tuple[3] - ang_vel_d_dot angular velocity desired derivative

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

    Rd = des_att_tuple[0]
    Rd_dot = des_att_tuple[1]
    ang_vel_d = des_att_tuple[2]
    ang_vel_d_dot = des_att_tuple[3]

    # determine error between command and current state
    eR = 1/2 * attitude.vee_map(Rd.T.dot(R) - R.T.dot(Rd))
    eW = ang_vel - R.T.dot(Rd).dot(ang_vel_d)
    # compute attitude input
    u_m = (-dum.kR*eR -dum.kW*eW + np.cross(ang_vel,dum.J.dot(ang_vel)) 
            -dum.J.dot( attitude.hat_map(ang_vel).dot(R.T).dot(Rd).dot(ang_vel_d)-
                R.T.dot(Rd).dot(ang_vel_d_dot)) - ext_moment)
    return u_m

def translation_controller(time, state, ext_force, dum, ast, des_tran_tuple):
    """SE(3) Translational Controller

    Parameters
    ----------
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
    ext_force : array_like (3,)
        External force in the inertial frame
    dum : Dumbbell Instance
        Instance of a dumbbell which defines the shape and MOI
    ast : Asteroid Instance
        Holds the asteroid model and polyhedron potential
    des_tran_tuple : Desired translational tuple
        des_tran_tuple [0] - Rd desired attitude
        des_tran_tuple [1] - Rd_dot desired attitude derivative
        des_tran_tuple [2] - ang_vel_d angular velocity
        des_tran_tuple [3] - ang_vel_d_dot angular velocity desired derivative

    """

    # extract the state
    pos = state[0:3] # location of the center of mass in the inertial frame
    vel = state[3:6] # vel of com in inertial frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame
    
    m = dum.m1 + dum.m2
    
    x_des = des_tran_tuple[0]
    xd_des = des_tran_tuple[1]
    xdd_des = des_tran_tuple[2]

    # x_des, xd_des, xdd_des = fixed_translation(time, ast=ast,final_pos=[0.550, 0, 0],
    #                                                       initial_pos=[2.550, 0, 0], 
    #                                                       descent_tf=3600)
    # compute the error
    ex = pos - x_des
    ev = vel - xd_des
    # compute the control
    u_f = -dum.kx * ex -dum.kv * ev - ext_force + m * xdd_des

    return u_f

def translation_controller_asteroid(time, state, ext_force, dum, ast, des_tran_tuple):
    """SE(3) Translational Controller
    
    This function determines the control input for the dumbbell with the equations
    of motion defined in the asteroid fixed frame. 
    Use this with eoms_relative

    Parameters
    ----------
    time : float
        Current time for simulation which is used in the desired attitude trajectory
    state : array_like (18,)
        numpy array defining the state of the dumbbell
        position - position of the center of mass wrt to the asteroid com
        and defined in the asteroid frame (3,)
        velocity - velocity of the center of mass wrt to teh asteroid com
        and defined in the asteroid frame (3,)
        R_b2i - rotation matrix which transforms vectors from the body
        frame to the asteroid frame (9,)
        angular_velocity - angular velocity of the body frame with respect
        to the inertial frame and defined in the asteroid frame (3,)
    ext_force : array_like (3,)
        External force in the asteroid frame
    dum : Dumbbell Instance
        Instance of a dumbbell which defines the shape and MOI
    ast : Asteroid Instance
        Holds the asteroid model and polyhedron potential
    des_tran_tuple : Desired translational tuple defined with respect to the asteroid frame
        des_tran_tuple [0] - Rd desired attitude
        des_tran_tuple [1] - Rd_dot desired attitude derivative
        des_tran_tuple [2] - ang_vel_d angular velocity
        des_tran_tuple [3] - ang_vel_d_dot angular velocity desired derivative

    """

    # extract the state
    pos = state[0:3] # location of the center of mass in the asteroid frame
    vel = state[3:6] # vel of com in asteroid frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to asteroid frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in asteroid frame
    
    m = dum.m1 + dum.m2
    Wa = ast.omega*np.array([0,0,1]) # angular velocity vector of asteroid

    x_des = des_tran_tuple[0]
    xd_des = des_tran_tuple[1]
    xdd_des = des_tran_tuple[2]

    # compute the error
    ex = pos - x_des
    ev = vel - xd_des
    # compute the control
    u_f = -dum.kx * ex - dum.kv * ev - ext_force + m * xdd_des + m * attitude.hat_map(Wa).dot(vel)

    return u_f

def attitude_controller_asteroid(time, state, ext_moment, dum, ast, des_att_tuple):
    r"""Geometric attitude controller on SO(3)

    This function will determine an attitude control input for a rigid
    spacecraft around an asteroid.
    The function is setup to work for a vehicle defined in the asteroid frame
    and the control input is assumed to be defined in the asteroid fixed frame.

    Parameters
    ----------
    time : float
        Current time for simulation which is used in the desired attitude trajectory
    state : array_like (18,)
        numpy array defining the state of the dumbbell
        position - position of the center of mass wrt to the asteroid com
        and defined in the asteroid frame (3,)
        velocity - velocity of the center of mass wrt to teh asteroid com
        and defined in the asteroid frame (3,)
        R_b2i - rotation matrix which transforms vectors from the body
        frame to the asteroid frame (9,)
        angular_velocity - angular velocity of the body frame with respect
        to the inertial frame and defined in the asteroid frame (3,)
    ext_moment : array_like (3,)
        External moment in the asteroid fixed frame
    dum : Dumbbell Instance
        Instance of a dumbbell which defines the shape and MOI
    ast : Asteroid Instance
        Holds the asteroid model and polyhedron potential
    des_att_tuple : Desired attitude tuple for asteroid frame dyanmics
        des_att_tuple[0] - Rd desired attitude
        des_att_tuple[1] - Rd_dot desired attitude derivative
        des_att_tuple[2] - ang_vel_d angular velocity
        des_att_tuple[3] - ang_vel_d_dot angular velocity desired derivative

    Returns
    -------
    u_m : array_like (3,)
        asteroid fixed control moment

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
    pos = state[0:3] # location of the center of mass in the asteroid frame
    vel = state[3:6] # vel of com in asteroid frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to asteriod frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in asteriod frame

    J = dum.J
    Jr = R.dot(J).dot(R.T)
    Wa = ast.omega*np.array([0,0,1]) # angular velocity vector of asteroid

    Rd = des_att_tuple[0]
    Rd_dot = des_att_tuple[1]
    ang_vel_d = des_att_tuple[2]
    ang_vel_d_dot = des_att_tuple[3]

    # determine error between command and current state
    eR = 1/2 * attitude.vee_map(Rd.T.dot(R) - R.T.dot(Rd))
    eW = ang_vel - R.T.dot(Rd).dot(ang_vel_d)
    # compute attitude input
    u_m = (-dum.kR*eR -dum.kW*eW + Jr.dot(attitude.hat_map(Wa).dot(ang_vel))
            -Jr.dot(attitude.hat_map(ang_vel).dot(R.T).dot(Rd).dot(ang_vel_d)
                -R.T.dot(Rd).dot(ang_vel_d_dot)) - ext_moment)
    return u_m

def asteroid_circumnavigate(time, tf=3600*6, loops=2):
    """Desired translation for circumnavaigation in asteroid frame

    This function will define the translational states for the dumbbell, 
    which is defined relative to the asteroid fixed frame.
    The states will define a path that will circumnavigate the asteroid in a 
    desired amount of time

    Parameters
    ----------
    time : float
        Current simulation time
    tf : int
        Time in seconds to do the entire trajectory.         
    loops : int
        Number of loops around asteroid to complete
    
    Returns
    -------
     
    """
    freq = 2 * np.pi * loops / tf
    dist = 3

    x_des = np.array([dist * np.cos(freq * time), dist * np.sin(freq * time), 0])
    xd_des = np.array([dist * freq * -np.sin(freq * time), dist * freq *
                       np.cos(freq * time), 0])
    xdd_des = np.array([dist * freq**2 * -np.cos(freq * time), dist * freq**2 *
                        -np.sin(freq * time), 0])

    return x_des, xd_des, xdd_des

def inertial_circumnavigate(time, tf=3600*6, loops=2):
    """Desired translation for circumnavigation in inertial frame

    This function will define the translational states for the dumbbell, 
    which is defined relative to the inertial fixed frame.
    The states will define a path that will circumnavigate the asteroid in a 
    desired amount of time

    Parameters
    ----------
    time : float
        Current simulation time
    tf : int
        Time in seconds to do the entire trajectory.         
    loops : int
        Number of loops around asteroid to complete
    
    Returns
    -------
     
    """
    freq = 2 * np.pi * loops / tf
    dist = 3

    x_des = np.array([dist * np.cos(freq * time), dist * np.sin(freq * time), 0])
    xd_des = np.array([dist * freq * -np.sin(freq * time), dist * freq *
                       np.cos(freq * time), 0])
    xdd_des = np.array([dist * freq**2 * -np.cos(freq * time), dist * freq**2 *
                        -np.sin(freq * time), 0])

    return x_des, xd_des, xdd_des

def asteroid_pointing(time, state, ast):
    """Point the spacecraft x axis toward the asteroid, in asteroid fixed frame

    Parameters
    ----------
    time : float
        Simulation time in seconds
    state : np.array (18,)
        pos - state[0:3] in km position of the dumbbell with respect to the
        asteroid and defined in the asteroid fixed frame
        vel - state[3:6] in km/sec is the velocity of dumbbell wrt the asteroid
        and defined in the asteroid fixed frame
        R - state[6:15] rotation matrix which converts vectors from the
        dumbbell frame to the asteroid frame
        w - state[15:18] rad/sec angular velocity of the dumbbell wrt inertial
        frame and defined in the asteroid frame

    Returns
    -------
    Rd : np.array (3,3)
        Desired attitude - spacecraft to asteroid fixed frame
    Rd_dot: np.array (3,3)
        Derivative of desired attitude
    ang_vel_d : np.array (3,)
        Desired angular velocity - spacecraft wrt inertial frame defined in the
        asteroid fixed frame
    ang_vel_d_dot : np.array (3,)
        Derivative of desired angular velocity

    """
    # extract out the states
    pos = state[0:3] # location of the center of mass in the asteroid frame
    vel = state[3:6] # vel of com in asteroid frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to asteroid frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in asteroid frame
    
    # compute the desired attitude to ensure that the body fixed x axis is pointing at the origin/asteroid
    # each column of the rotation matrix is the i-th body fixed axis as represented in the asteroid frame
    b1_des = - pos / np.linalg.norm(pos)
    b3_des = np.array([0, 0, 1]) - (np.array([0, 0, 1]).dot(b1_des) * b1_des)   # ensure the body z axis is always parallel with the asteroid/inertial z axis
    b3_des = b3_des / np.linalg.norm(b3_des)
    b2_des = np.cross(b3_des, b1_des)

    Rd = np.stack((b1_des, b2_des, b3_des), axis=1)

    Rd_dot = np.zeros((3,3))

    ang_vel_d = np.zeros(3) 
    ang_vel_d_dot = np.zeros(3) 

    return (Rd, Rd_dot, ang_vel_d, ang_vel_d_dot)

def attitude_traverse_then_land_controller(time, state, ext_moment, dum, ast):
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
    Rd, Rd_dot, ang_vel_d, ang_vel_d_dot = body_fixed_pointing_attitude(time, state)
    # determine error between command and current state
    eR = 1/2 * attitude.vee_map(Rd.T.dot(R) - R.T.dot(Rd))
    eW = ang_vel - R.T.dot(Rd).dot(ang_vel_d)
    # compute attitude input
    u_m = (-dum.kR*eR -dum.kW*eW + np.cross(ang_vel,dum.J.dot(ang_vel)) 
            -dum.J.dot( attitude.hat_map(ang_vel).dot(R.T).dot(Rd).dot(ang_vel_d)-
                R.T.dot(Rd).dot(ang_vel_d_dot)) - ext_moment)
    return u_m
def translation_traverse_then_land_controller(time, state, ext_force, dum, ast):
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
    x_des, xd_des, xdd_des = traverse_then_land_vertically(time, ast=ast,final_pos=[0.550, 0, 0],
                                                          initial_pos=[2.550, 0, 0], 
                                                          descent_tf=3600)
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

def body_fixed_pointing_attitude(time, state):
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

def traverse_then_land_vertically(time, ast, final_pos=[0.550, 0, 0], 
                                 initial_pos=[2.550, 0, 0], 
                                 descent_tf=3600):
    """Desired translational states for vertical landing on asteroid
   
    First the spacecraft will traverse horizontally in the equatorial plane
    over the asteroid fixed x axis, then descend vertically

    Inputs :
    --------
    
    """
    if time < descent_tf:
        # linear interpolation between whatever the current state is and the landing staging point
        omega = np.pi/2 / descent_tf
        inertial_pos = 2.550 * np.array([  np.sin(omega * time), -np.cos(omega * time), 0])
        inertial_vel = 2.550 * omega * np.array([np.cos(omega * time), np.sin(omega * time), 0])
        inertial_accel = 2.550 * omega**2 * np.array([-np.sin(omega * time), np.cos(omega *  time), 0])
    
    else:
        # rotation state of the asteroid - assume all simulations start with asteroid aligned with the inertial frame
        omega_ast = ast.omega
        omega_ast_dot = 0

        omega_ast_vec = np.array([0, 0, omega_ast])
        omega_ast_dot_vec = np.zeros(3)

        Ra2i = attitude.rot3(omega_ast * (time - descent_tf), 'c')
        # determine desired position and velocity in the body fixed frame at this current time input
        # we'll use a simple linear interpolation between initial and final states
        xslope =(final_pos[0] - initial_pos[0]) / (descent_tf)  # how long for teh descent
        xdes =  xslope * (time - descent_tf) + initial_pos[0]
        
        body_pos_des = np.array([xdes, 0, 0])
        body_vel_des = np.array([xslope, 0, 0])
        body_acc_des = np.zeros(3)
        # transform this body position/velocity into the inertial frame
        inertial_pos = Ra2i.dot(body_pos_des)
        inertial_vel = body_vel_des + np.cross(omega_ast_vec, body_pos_des)
        inertial_accel = body_acc_des + 2 * np.cross(omega_ast_vec, body_vel_des) + np.cross(omega_ast_vec, np.cross(omega_ast_vec, body_pos_des))

    # output
    return inertial_pos, inertial_vel, inertial_accel 

def linear_x_descent_translation(time, ast, final_pos=[0.550, 0, 0], 
                                 initial_pos=[2.550, 0, 0], 
                                 descent_tf=3600):
    """Desired translational states for vertical landing on asteroid
    
    Inputs :
    --------
    
    """
    # rotation state of the asteroid - assume all simulations start with asteroid aligned with the inertial frame
    omega_ast = ast.omega
    omega_ast_dot = 0

    omega_ast_vec = np.array([0, 0, omega_ast])
    omega_ast_dot_vec = np.zeros(3)

    Ra2i = attitude.rot3(omega_ast * time, 'c')
    # determine desired position and velocity in the body fixed frame at this current time input
    # we'll use a simple linear interpolation between initial and final states
    xslope =(final_pos[0] - initial_pos[0]) / (descent_tf) 
    xdes =  xslope * time + initial_pos[0]
    
    body_pos_des = np.array([xdes, 0, 0])
    body_vel_des = np.array([xslope, 0, 0])
    body_acc_des = np.zeros(3)
    # transform this body position/velocity into the inertial frame
    inertial_pos = Ra2i.dot(body_pos_des)
    inertial_vel = body_vel_des + np.cross(omega_ast_vec, body_pos_des)
    inertial_accel = body_acc_des + 2 * np.cross(omega_ast_vec, body_vel_des) + np.cross(omega_ast_vec, np.cross(omega_ast_vec, body_pos_des))

    # output
    return inertial_pos, inertial_vel, inertial_accel 
    
def fixed_attitude(time, state):
    """Desired attitude to ensure that the x axis is always pointing at the origin (asteroid)

    """
    # extract out the states
    pos = state[0:3] # location of the center of mass in the inertial frame
    vel = state[3:6] # vel of com in inertial frame
    R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
    ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame

    Rd = attitude.rot3(np.pi, 'c')
    Rd_dot = np.zeros_like(Rd)
    ang_vel_d = np.zeros(3) 
    ang_vel_d_dot = np.zeros(3) 

    return (Rd, Rd_dot, ang_vel_d, ang_vel_d_dot)

def fixed_translation(time, ast, final_pos=[0.550, 0, 0], 
                                 initial_pos=[2.550, 0, 0], 
                                 descent_tf=3600):
    """Desired translational states for vertical landing on asteroid
    
    Inputs :
    --------
    
    """
    # rotation state of the asteroid - assume all simulations start with asteroid aligned with the inertial frame
    omega_ast = ast.omega
    omega_ast_dot = 0

    omega_ast_vec = np.array([0, 0, omega_ast])
    omega_ast_dot_vec = np.zeros(3)

    inertial_pos = np.array([0, 2.5, 0])
    inertial_vel = np.zeros(3)
    inertial_accel = np.zeros(3) 

    # output
    return inertial_pos, inertial_vel, inertial_accel 

def approaching_translation(time, ast, final_pos=[0.550, 0, 0], 
                                 initial_pos=[2.550, 0, 0], 
                                 descent_tf=3600):
    """Desired translational states for vertical landing on asteroid
    
    Inputs :
    --------
    
    """
    # rotation state of the asteroid - assume all simulations start with asteroid aligned with the inertial frame
    omega_ast = ast.omega
    omega_ast_dot = 0

    omega_ast_vec = np.array([0, 0, omega_ast])
    omega_ast_dot_vec = np.zeros(3)

    inertial_pos = np.array([-0.0005 * time + initial_pos[0], 0, 0])
    inertial_vel = np.zeros(3)
    inertial_accel = np.zeros(3) 

    # output
    return inertial_pos, inertial_vel, inertial_accel 

