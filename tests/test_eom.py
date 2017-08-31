from __future__ import absolute_import, division, print_function, unicode_literals
from dynamics import dumbbell, asteroid, controller, eoms
from kinematics import attitude
import numpy as np
import pdb

angle = (2*np.pi- 0) * np.random.rand(1) + 0

# generate a random initial state that is outside of the asteroid
pos = np.random.rand(3)+np.array([2,2,2])
R = attitude.rot1(angle).reshape(9)
vel = np.random.rand(3)
ang_vel = np.random.rand(3)
t = np.random.rand()*100

state = np.hstack((pos,vel, R, ang_vel))
 
ast = asteroid.Asteroid('castalia',32)

dum = dumbbell.Dumbbell()

def test_eoms_controlled_relative_blender_ode_size():
    state_dot = eoms.eoms_controlled_relative_blender_ode(t, state, dum, ast)
    np.testing.assert_equal(state_dot.shape, (18,))
