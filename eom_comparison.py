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

def inertial2relative(time, state, ast, dum):
    # figure out transformation from inertial frame to relative frame
    Rast2int = np.zeros((3,3,inertial_time.shape[0]))
    Rint2ast = np.zeros((3,3,inertial_time.shape[0]))

    relative_state = np.zeros(state.shape)

    for ii,t in np.ndenumerate(inertial_time):
        Rast2int[:,:,ii] = attitude.rot3(ast.omega*t, 'c')[:,:,np.newaxis] # asteroid body frame to inertial frame
        Rint2ast[:,:,ii] = attitude.rot3(ast.omega*t, 'c').T[:,:,np.newaxis]

        Ra = np.squeeze(Rast2int[:,:,ii])
        # convert inertial states to relative states
        inertial_pos = np.squeeze(state[ii,0:3])
        inertial_vel = np.squeeze(state[ii,3:6])
        inertial_R = np.squeeze(state[ii,6:15].reshape(3,3))
        inertial_w = np.squeeze(state[ii,15:18])

        relative_pos = Ra.T.dot(inertial_pos)
        relative_vel = Ra.T.dot(inertial_vel)
        relative_R = Ra.T.dot(inertial_R).reshape(9)
        relative_w = Ra.T.dot(inertial_w)

        relative_state[ii,:] = np.hstack((relative_pos,relative_vel,relative_R,relative_w))

    return relative_state,Rast2int, Rint2ast


# load simulation data
inertial_filename = 'data/inertial_energy_castalia_64_1e5_inertial.npz'
relative_filename = 'data/relative_energy_castalia_64_1e5_relative.npz'
mode = 0

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

# convert inertial to relative frame
inertial2relative_state,_,_ = inertial2relative(inertial_time, inertial_state, ast, dum)

# plot/compare difference in relative frame


