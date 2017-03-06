import dynamics.dumbbell as dumbbell
import dynamics.asteroid as asteroid
import kinematics.attitude as attitude
import pdb
import numpy as np

class TestDumbbellInertial():

    ast = asteroid.Asteroid('castalia',32)
    dum = dumbbell.Dumbbell()
    angle = (2*np.pi- 0) * np.random.rand(1) + 0
    
    # generate a random initial state that is outside of the asteroid
    pos = np.random.rand(3)+np.array([2,2,2])
    R = attitude.rot1(angle).reshape(9)
    vel = np.random.rand(3)
    ang_vel = np.random.rand(3)
    t = np.random.rand()*100

    state = np.hstack((pos,vel, R, ang_vel))
    statedot = dum.eoms_inertial(state,t,ast)

    def test_inertia_symmetric(self):
        np.testing.assert_array_almost_equal(self.dum.J.T, self.dum.J)

    def test_eoms_inertial_size(self):
        np.testing.assert_allclose(self.statedot.shape, (18,))

    def test_eoms_inertial_pos_dot(self):
        np.testing.assert_allclose(self.statedot[0:3], self.vel)

    def test_eoms_inertial_R_dot(self):
        np.testing.assert_allclose(self.statedot[6:15], self.R.reshape(3,3).dot(attitude.hat_map(self.ang_vel)).reshape(9))
        
    def test_moment_of_inertia(self):
        np.testing.assert_allclose(self.dum.J, np.trace(self.dum.Jd)*np.eye(3,3) - self.dum.Jd)

class TestDumbbellRelative():
    ast = asteroid.Asteroid('castalia',32)
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

    def test_eoms_inertial_R_dot(self):
        np.testing.assert_allclose(self.statedot[6:15], )
        
    # def test_moment_of_inertia(self):
    #     np.testing.assert_allclose(self.dum.J, np.trace(self.dum.Jd)*np.eye(3,3) - self.dum.Jd)
