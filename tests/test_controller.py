from __future__ import absolute_import, division, print_function, unicode_literals
from dynamics import dumbbell, asteroid, controller
import kinematics.attitude as attitude
import pdb
import numpy as np

angle = (2*np.pi- 0) * np.random.rand(1) + 0

# generate a random initial state that is outside of the asteroid
pos = np.random.rand(3)+np.array([2,2,2])
R = attitude.rot1(angle).reshape(9)
vel = np.random.rand(3)
ang_vel = np.random.rand(3)
t = np.random.rand()*100

state = np.hstack((pos,vel, R, ang_vel))
 
ast = asteroid.Asteroid('castalia',32)

class TestInertialDesiredAttitude():
    
    dum = dumbbell.Dumbbell()
    alpha = np.random.rand()
    axis = np.array([1, 0, 0])
    Rd, Rd_dot, ang_vel_d, ang_vel_d_dot = controller.desired_attitude(1, alpha, axis)

    def test_desired_rotation_matrix_determinant(self):
        np.testing.assert_almost_equal(np.linalg.det(self.Rd), 1) 
    
    def test_desired_rotation_matrix_orthogonal(self):
        np.testing.assert_array_almost_equal(self.Rd.T.dot(self.Rd), 
                np.eye(3,3))

    def test_desired_attitude_satifies_kinematics(self):
        np.testing.assert_array_almost_equal(self.Rd_dot,
                self.Rd.dot(attitude.hat_map(self.ang_vel_d)))
 
class TestInertialDesiredAttitudeBodyFixedHovering():
    
    dum = dumbbell.Dumbbell()
    Rd, Rd_dot, ang_vel_d, ang_vel_d_dot = controller.body_fixed_hovering_attitude(1, state)

    def test_desired_rotation_matrix_determinant(self):
        np.testing.assert_almost_equal(np.linalg.det(self.Rd), 1) 
    
    def test_desired_rotation_matrix_orthogonal(self):
        np.testing.assert_array_almost_equal(self.Rd.T.dot(self.Rd), 
                np.eye(3,3))

    def test_desired_attitude_satifies_kinematics(self):
        np.testing.assert_array_almost_equal(self.Rd_dot,
                self.Rd.dot(attitude.hat_map(self.ang_vel_d)))

    def test_x_axis_anti_aligned_with_position_vector(self):
        dot_product = np.dot(self.Rd[:, 0], pos / np.linalg.norm(pos))
        np.testing.assert_almost_equal(dot_product, -1)

    def test_z_axis_aligned_with_inertial_z_vector(self):
        pass

    def test_y_axis_in_the_equatorial_plane(self):
        pass

class TestInertialAttitudeController():
    """Test the attitude controller for the inertial eoms
    """
    dum = dumbbell.Dumbbell()
    u_m = controller.attitude_controller(t, state, np.zeros(3), dum)

    def test_control_moment_size(self):
        np.testing.assert_equal(self.u_m.shape, (3,))

class TestDumbbellInertialTranslationalController():
    
    dum = dumbbell.Dumbbell()
    u_f = controller.translation_controller(t, state, np.zeros(3), dum)

    def test_control_force_size(self):
        np.testing.assert_equal(self.u_f.shape, (3,))
