"""
    Script to compare inertial and relative equations of motion

"""
import numpy as np

import matplotlib.pyplot as plt

import dynamics.asteroid as asteroid
import dynamics.dumbbell as dumbbell
import kinematics.attitude as attitude
import visualization.plotting as plotting

import inertial_driver as id
import relative_driver as rd

    
def load_data(inertial_filename, relative_filename, mode):
    """Load saved data and extract out the states

    """
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

def inertial_frame_comparison(ast_name='castalia', num_faces=64, tf=1e5, num_steps=1e6):
    """Compare EOMs in the inertial frame

    """
    initial_w = np.array([0.01, 0.0, 0.0])
    
    print("Running inertial EOMS")
    i_time, i_state = id.inertial_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)

    print("Running asteroid EOMs")
    r_time, r_state = rd.relative_eoms_driver(ast_name, num_faces, tf, num_steps, initial_w)

    ast = asteroid.Asteroid(ast_name,num_faces)
    dum = dumbbell.Dumbbell()

    # also compute and compare the energy behavior
    print("Computing inertial energy")
    i_KE, i_PE = dum.inertial_energy(i_time, i_state, ast)

    print("Computing asteroid energy")
    r_KE, r_PE = dum.relative_energy(r_time, r_state, ast)

    plotting.plot_energy(i_time, i_KE, i_PE)
    # also look at the animation of both and the converted form as well
    
    print("Plot comparison in the inertial frame")
    plotting.plot_inertial_comparison(r_time, i_time, r_state, i_state, ast, dum) 

    return 0

def asteroid_frame_comparison():
    """Compare EOMs in the asteroid frame

    """

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

if __name__ == '__main__':
    pass
