from __future__ import absolute_import, division, print_function, unicode_literals
import dynamics.dumbbell as dumbbell
import dynamics.asteroid as asteroid
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

class TestDumbbellInertial():

    dum = dumbbell.Dumbbell()
    statedot = dum.eoms_inertial(state,t,ast)

    def test_inertia_symmetric(self):
        np.testing.assert_array_almost_equal(self.dum.J.T, self.dum.J)

    def test_eoms_inertial_size(self):
        np.testing.assert_allclose(self.statedot.shape, (18,))

    def test_eoms_inertial_pos_dot(self):
        np.testing.assert_allclose(self.statedot[0:3], vel)

    def test_eoms_inertial_R_dot(self):
        np.testing.assert_allclose(self.statedot[6:15],R.reshape(3,3).dot(attitude.hat_map(ang_vel)).reshape(9))
        
    def test_moment_of_inertia(self):
        np.testing.assert_allclose(self.dum.J, np.trace(self.dum.Jd)*np.eye(3,3) - self.dum.Jd)

class TestDumbbellRelative():
    dum = dumbbell.Dumbbell()
    angle = (2*np.pi- 0) * np.random.rand(1) + 0
    
    # generate a random initial state that is outside of the asteroid
    pos = np.random.rand(3)+np.array([2,2,2])
    vel = np.random.rand(3)
    R = attitude.rot1(angle).reshape(9)
    ang_vel = np.random.rand(3)
    
    t = np.random.rand()*100

    state = np.hstack((pos,vel, R, ang_vel))
    statedot = dum.eoms_relative(state,t,ast)

    def test_relative_moment_of_inertia(self):
        R = self.R.reshape((3,3))
        Jr = R.dot(self.dum.J).dot(R.T)
        Jdr = R.dot(self.dum.Jd).dot(R.T)
        np.testing.assert_array_almost_equal(
            attitude.hat_map(Jr.dot(self.ang_vel)), 
            attitude.hat_map(self.ang_vel).dot(Jdr) + Jdr.dot(attitude.hat_map(self.ang_vel)))

    def test_eoms_relative_size(self):
        np.testing.assert_allclose(self.statedot.shape, (18,))

    def test_eoms_relative_R_dot(self):
        np.testing.assert_allclose(self.statedot[6:15].shape, (9,))
        
    # def test_moment_of_inertia(self):
    #     np.testing.assert_allclose(self.dum.J, np.trace(self.dum.Jd)*np.eye(3,3) - self.dum.Jd)

class TestDumbbellInertialDesiredAttitude():
    
    dum = dumbbell.Dumbbell()
    alpha = np.random.rand()
    axis = np.array([1, 0, 0])
    Rd, Rd_dot, ang_vel_d, ang_vel_d_dot = dum.desired_attitude(1, alpha, axis)

    def test_desired_rotation_matrix_determinant(self):
        np.testing.assert_almost_equal(np.linalg.det(self.Rd), 1) 
    
    def test_desired_rotation_matrix_orthogonal(self):
        np.testing.assert_array_almost_equal(self.Rd.T.dot(self.Rd), 
                np.eye(3,3))

    def test_desired_attitude_satifies_kinematics(self):
        np.testing.assert_array_almost_equal(self.Rd_dot,
                self.Rd.dot(attitude.hat_map(self.ang_vel_d)))
 
class TestDumbbellInertialAttitudeController():
    """Test the attitude controller for the inertial eoms
    """
    dum = dumbbell.Dumbbell()
    u_m = dum.attitude_controller(t, state, np.zeros(3))

    def test_control_moment_size(self):
        np.testing.assert_equal(self.u_m.shape, (3,))

class TestDumbbellInertialTranslationalController():
    
    dum = dumbbell.Dumbbell()
    u_f = dum.translation_controller(t, state, np.zeros(3))

    def test_control_force_size(self):
        np.testing.assert_equal(self.u_f.shape, (3,))
