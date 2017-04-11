"""
    Script to compare inertial and relative equations of motion

"""
import numpy as np
from scipy import integrate
import pdb

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
import kinematics.attitude as attitude
import plotting

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
        inertial_w = R_sc2ast.reshape((3, 3)).T.dot(ast_w)

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
    
def load_data(inertial_filename, relative_filename, mode):
    # load simulation data
    # inertial_filename = 'inertial_energy_test.npz'
    # relative_filename = 'relative_energy_test.npz'
    # mode = 0
    with np.load(inertial_filename, allow_pickle=True) as data:
        inertial_state = data['state']
        inertial_time = data['time']
        inertial_KE = data['KE']
        inertial_PE = data['PE']
        ast_name = data['ast_name'][()]
        num_faces = data['num_faces'][()]
        tf = data['tf'][()]
        num_steps = data['num_steps'][()]

    with np.load(relative_filename, allow_pickle=True) as data:
        relative_state = data['state']
        relative_time = data['time']
        relative_KE = data['KE']
        relative_PE = data['PE']
        relative_ast_name = data['ast_name'][()]
        relative_num_faces = data['num_faces'][()]
        relative_tf = data['tf'][()]
        relative_num_steps = data['num_steps'][()]

    # make sure we're dealing with the same simulation results or else the comparison is meaningless
    np.testing.assert_string_equal(relative_ast_name, ast_name)
    np.testing.assert_allclose(relative_num_faces, num_faces)
    np.testing.assert_allclose(relative_tf, tf)
    np.testing.assert_allclose(relative_num_steps, num_steps)
    np.testing.assert_allclose(relative_state.shape, inertial_state.shape)

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell()

    return relative_time, inertial_time, relative_state, inertial_state, ast, dum

def plot_relative_comparison(relative_time, inertial_time, relative_state, inertial_state, ast, dum):
    """Compare the EOMS in the relative frame (asteroid fixed frame)
    
    Inputs:
        relative_time
        inertial_time
        relative_state
        inertial_state
        ast
        dum

    Outputs:

    """
    # extract out the state
    inertial_pos = inertial_state[:,0:3]
    inertial_vel = inertial_state[:,3:6]
    inertial_R = inertial_state[:,6:15]
    inertial_w = inertial_state[:,15:18]

    relative_pos = relative_state[:,0:3]
    relative_vel = relative_state[:,3:6]
    relative_R = relative_state[:,6:15]
    relative_w = relative_state[:,15:18]

    # convert inertial to relative frame
    inertial2relative_state,_,_ = inertial2relative(inertial_time, inertial_state, ast, dum)

    i2r_pos = inertial2relative_state[:,0:3]
    i2r_vel = inertial2relative_state[:,3:6]
    i2r_R = inertial2relative_state[:,6:15]
    i2r_w = inertial2relative_state[:,15:18]

    # position comparison
    pos_fig, pos_axarr = plt.subplots(3,1, sharex=True)
    pos_axarr[0].plot(relative_time, relative_pos[:,0], label='Relative EOMs')
    pos_axarr[0].plot(inertial_time, i2r_pos[:,0], label='Transformed Inertial')
    pos_axarr[0].set_ylabel(r'$X$ (km)')
        
    pos_axarr[1].plot(relative_time, relative_pos[:,1], label='Relative EOMs')
    pos_axarr[1].plot(inertial_time, i2r_pos[:,1], label='Transformed Inertial')
    pos_axarr[1].set_ylabel(r'$Y$ (km)')
        
    pos_axarr[2].plot(relative_time, relative_pos[:,2], label='Relative EOMs')
    pos_axarr[2].plot(inertial_time, i2r_pos[:,2], label='Transformed Inertial')
    pos_axarr[2].set_ylabel(r'$Z$ (km)')
     
    pos_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Position Comparison')
    plt.legend()  

    posdiff_fig, posdiff_axarr = plt.subplots(3,1, sharex=True)
    posdiff_axarr[0].plot(relative_time, np.absolute(relative_pos[:,0]-i2r_pos[:,0]))
    posdiff_axarr[0].set_ylabel(r'$\Delta X$ (km)')
        
    posdiff_axarr[1].plot(relative_time, np.absolute(relative_pos[:,1]-i2r_pos[:,1]))
    posdiff_axarr[1].set_ylabel(r'$\Delta Y$ (km)')
        
    posdiff_axarr[2].plot(relative_time, np.absolute(relative_pos[:,2]-i2r_pos[:,2]))
    posdiff_axarr[2].set_ylabel(r'$\Delta Z$ (km)')
     
    posdiff_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Position Difference')

    # velocity comparison
    vel_fig, vel_axarr = plt.subplots(3,1, sharex=True)
    vel_axarr[0].plot(relative_time, relative_vel[:,0], label='Relative EOMs')
    vel_axarr[0].plot(inertial_time, i2r_vel[:,0], label='Transformed Inertial')
    vel_axarr[0].set_ylabel(r'$\dot X$ (km)')
        
    vel_axarr[1].plot(relative_time, relative_vel[:,1], label='Relative EOMs')
    vel_axarr[1].plot(inertial_time, i2r_vel[:,1], label='Transformed Inertial')
    vel_axarr[1].set_ylabel(r'$\dot Y$ (km)')
        
    vel_axarr[2].plot(relative_time, relative_vel[:,2], label='Relative EOMs')
    vel_axarr[2].plot(inertial_time, i2r_vel[:,2], label='Transformed Inertial')
    vel_axarr[2].set_ylabel(r'$\dot Z$ (km)')
     
    vel_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Velocity Comparison')
    plt.legend()

    veldiff_fig, veldiff_axarr = plt.subplots(3,1, sharex=True)
    veldiff_axarr[0].plot(relative_time, np.absolute(relative_vel[:,0]-i2r_vel[:,0]))
    veldiff_axarr[0].set_ylabel(r'$\Delta \dot X$ (km)')
        
    veldiff_axarr[1].plot(relative_time, np.absolute(relative_vel[:,1]-i2r_vel[:,1]))
    veldiff_axarr[1].set_ylabel(r'$\Delta \dot Y$ (km)')
        
    veldiff_axarr[2].plot(relative_time, np.absolute(relative_vel[:,2]-i2r_vel[:,2]))
    veldiff_axarr[2].set_ylabel(r'$\Delta \dot Z$ (km)')
     
    veldiff_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Velocity Difference')

    # angular velocity comparison
    angvel_fig, angvel_axarr = plt.subplots(3,1, sharex=True)
    angvel_axarr[0].plot(relative_time, relative_w[:,0], label='Relative EOMs')
    angvel_axarr[0].plot(inertial_time, i2r_w[:,0], label='Transformed Inertial')
    angvel_axarr[0].set_ylabel(r'$\dot \Omega_1$ (rad/sec)')
        
    angvel_axarr[1].plot(relative_time, relative_w[:,1], label='Relative EOMs')
    angvel_axarr[1].plot(inertial_time, i2r_w[:,1], label='Transformed Inertial')
    angvel_axarr[1].set_ylabel(r'$\dot \Omega_2$ (rad/sec)')
        
    angvel_axarr[2].plot(relative_time, relative_w[:,2], label='Relative EOMs')
    angvel_axarr[2].plot(inertial_time, i2r_w[:,2], label='Transformed Inertial')
    angvel_axarr[2].set_ylabel(r'$\dot \Omega_3$ (rad/sec)')
     
    angvel_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Angular Velocity Comparison')
    plt.legend()

    angveldiff_fig, angveldiff_axarr = plt.subplots(3,1, sharex=True)
    angveldiff_axarr[0].plot(relative_time, np.absolute(relative_w[:,0]-i2r_w[:,0]))
    angveldiff_axarr[0].set_ylabel(r'$\Delta \dot \Omega$ (rad/sec)')
        
    angveldiff_axarr[1].plot(relative_time, np.absolute(relative_w[:,1]-i2r_w[:,1]))
    angveldiff_axarr[1].set_ylabel(r'$\Delta \dot \Omega_2$ (rad/sec)')
        
    angveldiff_axarr[2].plot(relative_time, np.absolute(relative_w[:,2]-i2r_w[:,2]))
    angveldiff_axarr[2].set_ylabel(r'$\Delta \dot \Omega_3$ (rad/sec)')
     
    angveldiff_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Angular Velocity Difference')

    # attitude matrix comparison
    att_fig, att_axarr = plt.subplots(3,3, sharex=True, sharey=True)
    plt.suptitle('Rotation Matrix')
    for ii in range(9):
        row, col = np.unravel_index(ii, [3,3])
        att_axarr[row,col].plot(relative_time, relative_R[:,ii])
        att_axarr[row,col].plot(relative_time, i2r_R[:,ii])

    # attitude matrix difference
    attdiff_fig, attdiff_axarr = plt.subplots(3,3, sharex=True, sharey=True)
    plt.suptitle('Rotation Matrix Difference')
    for ii in range(9):
        row, col = np.unravel_index(ii, [3,3])
        attdiff_axarr[row,col].plot(relative_time, np.absolute(relative_R[:,ii]-i2r_R[:,ii]))

    plt.show()

    return 0

def plot_inertial_comparison(ast_time, body_time, ast_state, body_state, ast, dum):
    """Compare the EOMS in the inertial frame, everything transformed to the inertial frame
    
    Inputs:
        ast_time - time vector from eoms_relative
        body_time - time vector from eoms_inertial
        ast_state - state vector from eoms_relative
        body_state - state vector from eoms_inertial
        ast - instance of Asteroid class
        dum - instance of Dumbbell class

    Outputs:

    """
    # convert simulations into the inertial frame    
    inertial_state = body2inertial(body_time, body_state, ast, dum) 
    ast2inertial_state,_,_ = ast2inertial(ast_time, ast_state, ast, dum)
    
     # extract out the states
    inertial_pos = inertial_state[:,0:3]
    inertial_vel = inertial_state[:,3:6]
    inertial_R_sc2inertial = inertial_state[:,6:15]
    inertial_w = inertial_state[:,15:18]
   
    a2i_pos = ast2inertial_state[:,0:3]
    a2i_vel = ast2inertial_state[:,3:6]
    a2i_R = ast2inertial_state[:,6:15]
    a2i_w = ast2inertial_state[:,15:18]
    
    # position comparison
    pos_fig, pos_axarr = plt.subplots(3,1, sharex=True)
    pos_axarr[0].plot(body_time, inertial_pos[:,0], label='Inertial EOMs')
    pos_axarr[0].plot(ast_time, a2i_pos[:,0], label='Transformed Relative')
    pos_axarr[0].set_ylabel(r'$X$ (km)')
        
    pos_axarr[1].plot(body_time, inertial_pos[:,1], label='Inertial EOMs')
    pos_axarr[1].plot(ast_time, a2i_pos[:,1], label='Transformed Relative')
    pos_axarr[1].set_ylabel(r'$Y$ (km)')
        
    pos_axarr[2].plot(body_time, inertial_pos[:,2], label='Inertial EOMs')
    pos_axarr[2].plot(ast_time, a2i_pos[:,2], label='Transformed Relative')
    pos_axarr[2].set_ylabel(r'$Z$ (km)')
     
    pos_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Position Comparison')
    plt.legend()  

    posdiff_fig, posdiff_axarr = plt.subplots(3,1, sharex=True)
    posdiff_axarr[0].plot(body_time, np.absolute(inertial_pos[:,0]-a2i_pos[:,0]))
    posdiff_axarr[0].set_ylabel(r'$\Delta X$ (km)')
        
    posdiff_axarr[1].plot(body_time, np.absolute(inertial_pos[:,1]-a2i_pos[:,1]))
    posdiff_axarr[1].set_ylabel(r'$\Delta Y$ (km)')
        
    posdiff_axarr[2].plot(body_time, np.absolute(inertial_pos[:,2]-a2i_pos[:,2]))
    posdiff_axarr[2].set_ylabel(r'$\Delta Z$ (km)')
     
    posdiff_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Position Difference')

    # velocity comparison
    vel_fig, vel_axarr = plt.subplots(3,1, sharex=True)
    vel_axarr[0].plot(body_time, inertial_vel[:,0], label='inertial EOMs')
    vel_axarr[0].plot(ast_time, a2i_vel[:,0], label='Transformed relative')
    vel_axarr[0].set_ylabel(r'$\dot X$ (km)')
        
    vel_axarr[1].plot(body_time, inertial_vel[:,1], label='inertial EOMs')
    vel_axarr[1].plot(ast_time, a2i_vel[:,1], label='Transformed relative')
    vel_axarr[1].set_ylabel(r'$\dot Y$ (km)')
        
    vel_axarr[2].plot(body_time, inertial_vel[:,2], label='inertial EOMs')
    vel_axarr[2].plot(ast_time, a2i_vel[:,2], label='Transformed relative')
    vel_axarr[2].set_ylabel(r'$\dot Z$ (km)')
     
    vel_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Velocity Comparison')
    plt.legend()

    veldiff_fig, veldiff_axarr = plt.subplots(3,1, sharex=True)
    veldiff_axarr[0].plot(body_time, np.absolute(inertial_vel[:,0]-a2i_vel[:,0]))
    veldiff_axarr[0].set_ylabel(r'$\Delta \dot X$ (km)')
        
    veldiff_axarr[1].plot(body_time, np.absolute(inertial_vel[:,1]-a2i_vel[:,1]))
    veldiff_axarr[1].set_ylabel(r'$\Delta \dot Y$ (km)')
        
    veldiff_axarr[2].plot(body_time, np.absolute(inertial_vel[:,2]-a2i_vel[:,2]))
    veldiff_axarr[2].set_ylabel(r'$\Delta \dot Z$ (km)')
     
    veldiff_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Velocity Difference')

    # angular velocity comparison
    angvel_fig, angvel_axarr = plt.subplots(3,1, sharex=True)
    angvel_axarr[0].plot(body_time, inertial_w[:,0], label='Inertial EOMs')
    angvel_axarr[0].plot(ast_time, a2i_w[:,0], label='Transformed Relative')
    angvel_axarr[0].set_ylabel(r'$\dot \Omega_1$ (rad/sec)')
        
    angvel_axarr[1].plot(body_time, inertial_w[:,1], label='Inertial EOMs')
    angvel_axarr[1].plot(ast_time, a2i_w[:,1], label='Transformed Relative')
    angvel_axarr[1].set_ylabel(r'$\dot \Omega_2$ (rad/sec)')
        
    angvel_axarr[2].plot(body_time, inertial_w[:,2], label='Inertial EOMs')
    angvel_axarr[2].plot(ast_time, a2i_w[:,2], label='Transformed Relative')
    angvel_axarr[2].set_ylabel(r'$\dot \Omega_3$ (rad/sec)')
     
    angvel_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Angular Velocity Comparison')
    plt.legend()

    angveldiff_fig, angveldiff_axarr = plt.subplots(3,1, sharex=True)
    angveldiff_axarr[0].plot(body_time, np.absolute(inertial_w[:,0]-a2i_w[:,0]))
    angveldiff_axarr[0].set_ylabel(r'$\Delta \dot \Omega$ (rad/sec)')
        
    angveldiff_axarr[1].plot(body_time, np.absolute(inertial_w[:,1]-a2i_w[:,1]))
    angveldiff_axarr[1].set_ylabel(r'$\Delta \dot \Omega_2$ (rad/sec)')
        
    angveldiff_axarr[2].plot(body_time, np.absolute(inertial_w[:,2]-a2i_w[:,2]))
    angveldiff_axarr[2].set_ylabel(r'$\Delta \dot \Omega_3$ (rad/sec)')
     
    angveldiff_axarr[2].set_xlabel('Time (sec)')
    plt.suptitle('Angular Velocity Difference')

    # attitude matrix comparison
    att_fig, att_axarr = plt.subplots(3,3, sharex=True, sharey=True)
    plt.suptitle('Rotation Matrix')
    for ii in range(9):
        row, col = np.unravel_index(ii, [3,3])
        att_axarr[row,col].plot(body_time, inertial_R_sc2inertial[:,ii])
        att_axarr[row,col].plot(ast_time, a2i_R[:,ii])

    # attitude matrix difference
    attdiff_fig, attdiff_axarr = plt.subplots(3,3, sharex=True, sharey=True)
    plt.suptitle('Rotation Matrix Difference')
    for ii in range(9):
        row, col = np.unravel_index(ii, [3,3])
        attdiff_axarr[row,col].plot(body_time, np.absolute(inertial_R_sc2inertial[:,ii]-a2i_R[:,ii]))

    plt.show()

    return 0

def relative_frame_comparision():
    """Compare the EOMs in the asteroid fixed frame

    Inputs:

    Outputs:

    Dependencies:


    """
    import inertial_driver as id
    import relative_driver as rd

    ast_name = 'castalia'
    num_faces = 64
    tf = 1e3
    num_steps = 1e3

    i_time, i_state = id.inertial_eoms_driver(ast_name, num_faces, tf, num_steps)
    r_time, r_state = rd.relative_eoms_driver(ast_name, num_faces, tf, num_steps)

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell()

    # also compute and compare the energy behavior

    # also look at the animation of both and the converted form as well
    
    plot_relative_comparison(r_time, i_time, r_state, i_state, ast, dum) 

    return 0
    
def inertial_frame_comparison():
    """Compare EOMs in the inertial frame

    """
    import inertial_driver as id
    import relative_driver as rd

    ast_name = 'castalia'
    num_faces = 64
    tf = 1e4
    num_steps = 1e5
    
    initial_w = np.array([0.0, 0.01, 0.0])

    i_time, i_state = id.inertial_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)
    r_time, r_state = rd.relative_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell()

    # also compute and compare the energy behavior
    i_KE, i_PE = dum.inertial_energy(i_time, i_state, ast)
    r_KE, r_PE = dum.relative_energy(r_time, r_state, ast)

    plotting.plot_energy(i_time, i_KE, i_PE)
    # also look at the animation of both and the converted form as well
    
    plot_inertial_comparison(r_time, i_time, r_state, i_state, ast, dum) 

    return 0

if __name__ == '__main__':
    pass
