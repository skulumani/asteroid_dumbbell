"""Conversion of states between the various frames

Use this module to conver the results of the simulations between the three frames of interest.
It will be able to convert between:
    Inertial frame - Inertial frame which is centered at the asteroid and fixed with respect 
    to the stars
    Asteroid frame - rotating frame which is fixed at the center of the asteroid and aligned with
    the principle axes of the body
    Dumbbell frame - frame located at the center of mass of the dumbbell and aligned with the 
    principle moments of inertia

"""
import numpy as np

import kinematics.attitude as attitude

def inertial2ast(time, state, ast, dum):
    """Convert inertial state to the asteroid fixed frame
    
    This will convert a state vector which is defined in the inertial frame
    into the equivalent representation in the rotating asteroid frame. 

    Inputs:
        time - nx1 array of time stamps for the state vector
        state - nx18 array of the state vector defined as follows:
            inertial_pos - state[:, 0:3] is the position of the body with respect to the asteroid 
                center and defined in the inertial frame
            inertial_vel - state[:, 4:6] is the velocity of the sc with respect to the asteroid 
                center of mass and defined in the inertial frame
            R_sc2int - state[:, 6:15] the rotation matrix which transforms vectors from the sc frame
                to the inertial frame
            inertial_w - state[:, 15:18] the angular velocity of the sc with respect to the inertial
                frame and defined in the inertial frame
        ast - Instance of Asteroid class
        dum - Instance of Dumbbell class

    Outputs:
        ast_state - the state as represented in the rotating asteroid fixed frame
            ast_pos - ast_state[:, 0:3] is the position of the sc with respect to the asteroid
                fixed frame and represented in the asteroid frame
            ast_vel - ast_state[:, 3:6] is the velocity of the sc with respect to the asteroid and 
                represented in the asteroid fixed frame
            ast_R_sc2ast - ast_state[:, 6:15] is the transformation from the sc frame to the asteroid
                frame
            ast_w - ast_state[:, 15:18] is the angular velocity of the sc with respect ot the inertial 
                frame and defined in the asteroid rotating frame

    """

    # figure out transformation from inertial frame to relative frame
    Rast2int = np.zeros((3,3,time.shape[0]))
    Rint2ast = np.zeros((3,3,time.shape[0]))

    ast_state = np.zeros(state.shape)

    for ii,t in np.ndenumerate(time):
        Rast2int[:,:,ii] = attitude.rot3(ast.omega*t, 'c')[:,:,np.newaxis] # asteroid body frame to inertial frame
        Rint2ast[:,:,ii] = attitude.rot3(ast.omega*t, 'c').T[:,:,np.newaxis]

        Ra = np.squeeze(Rast2int[:,:,ii])
        # convert inertial states to relative states
        inertial_pos = np.squeeze(state[ii,0:3])
        inertial_vel = np.squeeze(state[ii,3:6])
        R_sc2int = np.squeeze(state[ii,6:15].reshape(3,3))
        inertial_w = np.squeeze(state[ii,15:18])

        ast_pos = Ra.T.dot(inertial_pos)
        ast_vel = Ra.T.dot(inertial_vel)
        ast_R_sc2ast = Ra.T.dot(R_sc2int).reshape(9)
        ast_w = ast_R_sc2ast.reshape((3,3)).dot(inertial_w)

        ast_state[ii,:] = np.hstack((ast_pos, ast_vel, ast_R_sc2ast, ast_w))

    return ast_state,Rast2int, Rint2ast

def ast2inertial(time, state, ast, dum):
    """Convert from the asteroid frame to the inertial frame
     
    This will convert a state vector which is defined in the asteroid frame
    into the equivalent representation in the inertial frame. 

    Inputs:
        time - nx1 array of time stamps for the state vector
        state - nx18 array of the state vector defined as follows:
            ast_pos - state[:, 0:3] is the position of the body with respect to the asteroid 
                center and defined in the asteroid frame
            ast_vel - state[:, 4:6] is the velocity of the sc with respect to the asteroid 
                center of mass and defined in the asteroid frame
            R_sc2ast - state[:, 6:15] the rotation matrix which transforms vectors from the sc frame
                to the asteroid frame
            ast_w - state[:, 15:18] the angular velocity of the sc with respect to the inertial
                frame and defined in the asteroid frame
        ast - Instance of Asteroid class
        dum - Instance of Dumbbell class

    Outputs:
        inertial_state - the state as represented in the rotating asteroid fixed frame
            inertial_pos - inertial_state[:, 0:3] is the position of the sc with respect to the asteroid
                fixed frame and represented in the inertial frame
            inertial_vel - inertial_state[:, 3:6] is the velocity of the sc with respect to the asteroid and 
                represented in the inertial fixed frame
            inertial_R_sc2int - inertial_state[:, 6:15] is the transformation from the sc frame to the inertial
                frame
            inertial_w - inertial_state[:, 15:18] is the angular velocity of the sc with respect to the inertial 
                frame and defined in the inertial frame


    """

    # transformation between asteroid fixed frame and inertial frame
    # figure out transformation from inertial frame to relative frame
    Rast2int = np.zeros((3, 3, time.shape[0]))
    Rint2ast = np.zeros((3, 3, time.shape[0]))

    inertial_state = np.zeros(state.shape)

    for ii, t in np.ndenumerate(time):
        Rast2int[:, :, ii] = attitude.rot3(ast.omega*t, 'c')[:, :, np.newaxis] # asteroid body frame to inertial frame
        Rint2ast[:, :, ii] = attitude.rot3(ast.omega*t, 'c').T[:, :, np.newaxis]

        Ra = np.squeeze(Rast2int[:, :, ii])
        # convert the relative state to the inertial frame
        ast_pos = np.squeeze(state[ii, 0:3])
        ast_vel = np.squeeze(state[ii, 3:6])
        R_sc2ast = np.squeeze(state[ii, 6:15].reshape(3, 3))
        ast_w = np.squeeze(state[ii, 15:18])

        inertial_pos = Ra.dot(ast_pos)
        inertial_vel = Ra.dot(ast_vel)
        inertial_R_sc2int = Ra.dot(R_sc2ast).reshape(9)
        inertial_w = Ra.dot(ast_w)

        inertial_state[ii, :] = np.hstack((inertial_pos, inertial_vel, inertial_R_sc2int, inertial_w))

    return inertial_state, Rast2int, Rint2ast

def body2inertial(time, state, ast, dum):
    """Convert SC state to inertial state
 
    This will convert a state vector which is defined in the spacecraft frame
    into the equivalent representation in the inertial frame. 

    Inputs:
        time - nx1 array of time stamps for the state vector
        state - nx18 array of the state vector defined as follows:
            inertial_pos - state[:, 0:3] is the position of the body with respect to the asteroid 
                center and defined in the inertial frame
            inertial_vel - state[:, 4:6] is the velocity of the sc with respect to the asteroid 
                center of mass and defined in the inertial frame
            R_sc2int - state[:, 6:15] the rotation matrix which transforms vectors from the sc frame
                to the inertial frame
            body_w - state[:, 15:18] the angular velocity of the sc with respect to the inertial
                frame and defined in the spacecraft frame
        ast - Instance of Asteroid class
        dum - Instance of Dumbbell class

    Outputs:
        inertial_state - the state as represented in the rotating inertial frame
            inertial_pos - inertial_state[:, 0:3] is the position of the sc with respect to the asteroid
                fixed frame and represented in the inertial frame
            inertial_vel - inertial_state[:, 3:6] is the velocity of the sc with respect to the asteroid and 
                represented in the inertial fixed frame
            inertial_R_sc2int - inertial_state[:, 6:15] is the transformation from the sc frame to the inertial
                frame
            inertial_w - inertial_state[:, 15:18] is the angular velocity of the sc with respect to the inertial 
                frame and defined in the inertial frame


    """
    inertial_state = np.zeros(state.shape)

    for ii, t in np.ndenumerate(time):

        # convert the relative state to the inertial frame
        inertial_pos = np.squeeze(state[ii, 0:3])
        inertial_vel = np.squeeze(state[ii, 3:6])
        R_sc2int = np.squeeze(state[ii, 6:15])
        body_w = np.squeeze(state[ii, 15:18])

        inertial_w = R_sc2int.reshape((3, 3)).dot(body_w)

        inertial_state[ii, :] = np.hstack((inertial_pos, inertial_vel,R_sc2int, inertial_w))

    return inertial_state 

def body2ast(time, state, ast, dum):
    """Convert state from the body frame to the asteroid fixed frame

    This function will convert the state output of eoms_inertial to the asteroid fixed frame.
    This wil allow for comparision with the state from eoms_relative.

    Inputs:
        time - nx1 array of time stamps for the state vector
        state - nx18 array of the state vector defined as follows:
            inertial_pos - state[:, 0:3] is the position of the body with respect to the asteroid 
                center and defined in the inertial frame
            inertial_vel - state[:, 4:6] is the velocity of the sc with respect to the asteroid 
                center of mass and defined in the inertial frame
            R_sc2int - state[:, 6:15] the rotation matrix which transforms vectors from the sc frame
                to the inertial frame
            body_w - state[:, 15:18] the angular velocity of the sc with respect to the inertial
                frame and defined in the spacecraft frame
        ast - Instance of Asteroid class
        dum - Instance of Dumbbell class

    Outputs:
        ast_state - the state as represented in the rotating asteroid frame
            ast_pos - ast_state[:, 0:3] is the position of the sc with respect to the asteroid
                fixed frame and represented in the asteroid frame
            ast_vel - ast_state[:, 3:6] is the velocity of the sc with respect to the asteroid and 
                represented in the asteroid fixed frame
            ast_R_sc2ast - ast_state[:, 6:15] is the transformation from the sc frame to the asteroid
                frame
            ast_w - ast_state[:, 15:18] is the angular velocity of the sc with respect to the inertial 
                frame and defined in the asteroid frame
    """
    # transformation between asteroid fixed frame and inertial frame
    # figure out transformation from inertial frame to relative frame
    Rast2int = np.zeros((3, 3, time.shape[0]))
    Rint2ast = np.zeros((3, 3, time.shape[0]))

    ast_state = np.zeros(state.shape)

    for ii, t in np.ndenumerate(time):
        Rast2int[:, :, ii] = attitude.rot3(ast.omega*t, 'c')[:, :, np.newaxis] # asteroid body frame to inertial frame
        Rint2ast[:, :, ii] = attitude.rot3(ast.omega*t, 'c').T[:, :, np.newaxis]

        Ra = np.squeeze(Rast2int[:, :, ii])
        # convert the relative state to the inertial frame
        body_pos = np.squeeze(state[ii, 0:3])
        body_vel = np.squeeze(state[ii, 3:6])
        R_sc2int = np.squeeze(state[ii, 6:15].reshape(3, 3))
        body_w = np.squeeze(state[ii, 15:18])

        ast_pos = Ra.T.dot(body_pos)
        ast_vel = Ra.T.dot(body_vel)
        ast_R_sc2ast = Ra.T.dot(R_sc2int).reshape(9)
        ast_w = ast_R_sc2ast.reshape((3, 3)).dot(body_w)

        ast_state[ii, :] = np.hstack((ast_pos, ast_vel, ast_R_sc2ast, ast_w))

    return ast_states

def eoms_inertial_to_inertial(time, state, ast, dum):
    """Transform the simulation result of the eoms_inertial into the inertial frame
    
    The eoms_inertial are not completely defined in the inertial frame.
    The angular velocity needs to be converted. This function eases that issue by simply accpeting
    the entire state output from eoms_inertial and transforming it.

    Parameters
    ----------
    time : nx1 numpy array
        time array of the simulation
    state : nx18 numpy array
        pos - state[0:3] in km position of the dumbbell with respect to the asteroid and defined in the asteroid fixed frame
        lin_mom - state[3:6] in kg km/sec is the linear momentum of dumbbell wrt the asteroid and defined in the asteroid fixed frame
        R - state[6:15] rotation matrix which converts vectors from the dumbbell frame to the asteroid frame
        ang_mom - state[15:18] J rad/sec angular momentum of the dumbbell wrt inertial frame and defined in the asteroid frame
    ast : Instance of Asteroid class
    dum : Instance of Dumbbell class
    
    Returns
    -------
        inertial_state : nx18 numpy array 
            inertial_pos - inertial_state[:, 0:3] is the position of the sc with respect to the asteroid
                fixed frame and represented in the inertial frame
            inertial_vel - inertial_state[:, 3:6] is the velocity of the sc with respect to the asteroid and 
                represented in the inertial fixed frame
            inertial_R_sc2int - inertial_state[:, 6:15] is the transformation from the sc frame to the inertial
                frame
            inertial_w - inertial_state[:, 15:18] is the angular velocity of the sc with respect to the inertial 
                frame and defined in the inertial frame
    """
    inertial_state = np.zeros(state.shape)

    # logic to check if the input is only a single element array
    if state.ndim == 1:
        inertial_pos = np.squeeze(state[0:3])
        inertial_vel = np.squeeze(state[3:6])
        R_sc2int = state[6:15]
        body_w = state[15:18]

        inertial_w = R_sc2int.reshape((3, 3)).dot(body_w)

        inertial_state = np.hstack((inertial_pos, inertial_vel,R_sc2int, inertial_w))
    elif state.ndim == 2:

        for ii, tup in enumerate(zip(time, state)):
            t, state_ii = tup
            # convert the relative state to the inertial frame
            inertial_pos = np.squeeze(state_ii[0:3])
            inertial_vel = np.squeeze(state_ii[3:6])
            R_sc2int = np.squeeze(state_ii[6:15])
            body_w = np.squeeze(state_ii[15:18])

            inertial_w = R_sc2int.reshape((3, 3)).dot(body_w)

            inertial_state[ii, :] = np.hstack((inertial_pos, inertial_vel,R_sc2int, inertial_w))
    else:
        print("Some kind of crazy error")
        return 2

    return inertial_state 

def eoms_hamilton_relative_to_inertial(time, state, ast, dum):
    """Convert the relative eoms to the inertial frame

    This function will convert the output from the eoms_hamilton_relative into the inertial frame.
    Since the equations are defined in the hamiltonian form first the inverse legendre transform 
    is used followed by a transformation into the inertial frame. 

    The output of this function along with eoms_inertial_to_inertial should be in the same frame
    and allows for easy comparison.

    Parameters
    ----------
    time : nx1 numpy array
        time array of the simulation
    state : nx18 numpy array
        pos - state[0:3] in km position of the dumbbell with respect to the asteroid and defined in the asteroid fixed frame
        lin_mom - state[3:6] in kg km/sec is the linear momentum of dumbbell wrt the asteroid and defined in the asteroid fixed frame
        R - state[6:15] rotation matrix which converts vectors from the dumbbell frame to the asteroid frame
        ang_mom - state[15:18] J rad/sec angular momentum of the dumbbell wrt inertial frame and defined in the asteroid frame
    ast - Instance of Asteroid class
    dum - Instance of Dumbbell class
    
    Returns
    -------
        inertial_state : nx18 numpy array 
            inertial_pos - inertial_state[:, 0:3] is the position of the sc with respect to the asteroid
                fixed frame and represented in the inertial frame
            inertial_vel - inertial_state[:, 3:6] is the velocity of the sc with respect to the asteroid and 
                represented in the inertial fixed frame
            inertial_R_sc2int - inertial_state[:, 6:15] is the transformation from the sc frame to the inertial
                frame
            inertial_w - inertial_state[:, 15:18] is the angular velocity of the sc with respect to the inertial 
                frame and defined in the inertial frame
    """
    if state.ndim == 1:
        # first do the inverse legendre transform
        rel_lin_mom = state[3:6]
        rel_ang_mom = state[15:18]

        rh_vel = rel_lin_mom / (dum.m1 + dum.m2)
        rh_ang_vel = np.zeros_like(rel_ang_mom)

        Rast2int = attitude.rot3(ast.omega*time, 'c') # asteroid body frame to inertial frame
        Rint2ast = attitude.rot3(ast.omega*time, 'c')
        Ra = Rast2int 

        R = state[6:15].reshape((3,3))    

        Jr = R.dot(dum.J).dot(R.T)
        rh_ang_vel = np.linalg.inv(Jr).dot(rel_ang_mom)

        rh_state_conv = np.hstack((state[0:3], rh_vel, R.reshape(9), rh_ang_vel))
        # convert the relative state to the inertial frame
        ast_pos = np.squeeze(rh_state_conv[0:3])
        ast_vel = np.squeeze(rh_state_conv[3:6])
        R_sc2ast = np.squeeze(rh_state_conv[6:15].reshape(3, 3))
        ast_w = np.squeeze(rh_state_conv[15:18])

        inertial_pos = Ra.dot(ast_pos)
        inertial_vel = Ra.dot(ast_vel)
        inertial_R_sc2int = Ra.dot(R_sc2ast).reshape(9)
        inertial_w = Ra.dot(ast_w)

        inertial_state = np.hstack((inertial_pos, inertial_vel, inertial_R_sc2int, inertial_w))

    elif state.ndim == 2:
        # first do the inverse legendre transform
        rel_lin_mom = state[:, 3:6]
        rel_ang_mom = state[:, 15:18]

        rh_vel = rel_lin_mom / (dum.m1 + dum.m2)
        rh_ang_vel = np.zeros_like(rel_ang_mom)

        # convert from the relative asteroid frame into the inertial frame
        Rast2int = np.zeros((3, 3, time.shape[0]))
        Rint2ast = np.zeros((3, 3, time.shape[0]))
        
        rh_state_conv = np.zeros(state.shape)
        inertial_state = np.zeros(state.shape)

        for ii, t in np.ndenumerate(time):
            Rast2int[:, :, ii] = attitude.rot3(ast.omega*t, 'c')[:, :, np.newaxis] # asteroid body frame to inertial frame
            Rint2ast[:, :, ii] = attitude.rot3(ast.omega*t, 'c').T[:, :, np.newaxis]
            Ra = np.squeeze(Rast2int[:, :, ii])

            R = state[ii, 6:15].reshape((3,3))    

            Jr = R.dot(dum.J).dot(R.T)
            rh_ang_vel[ii, :] = np.linalg.inv(Jr).dot(np.squeeze(rel_ang_mom[ii, :]))

            rh_state_conv[ii, :] = np.hstack((state[ii, 0:3], rh_vel[ii, :], R.reshape((1,9)), rh_ang_vel[ii, :]))
            # convert the relative state to the inertial frame
            ast_pos = np.squeeze(rh_state_conv[ii, 0:3])
            ast_vel = np.squeeze(rh_state_conv[ii, 3:6])
            R_sc2ast = np.squeeze(rh_state_conv[ii, 6:15].reshape(3, 3))
            ast_w = np.squeeze(rh_state_conv[ii, 15:18])

            inertial_pos = Ra.dot(ast_pos)
            inertial_vel = Ra.dot(ast_vel)
            inertial_R_sc2int = Ra.dot(R_sc2ast).reshape(9)
            inertial_w = Ra.dot(ast_w)

            inertial_state[ii, :] = np.hstack((inertial_pos, inertial_vel, inertial_R_sc2int, inertial_w))

    return inertial_state

