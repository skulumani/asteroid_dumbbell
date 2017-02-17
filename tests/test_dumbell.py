import dynamics.dumbbell as dumbbell
import dynamics.asteroid as asteroid
import attitude_ref.attitude as attitude
import pdb
import numpy as np

class TestDumbbell():

    ast = asteroid.Asteroid('castalia',32)
    dum = dumbbell.Dumbbell()
    angle = (2*np.pi- 0) * np.random.rand(1) + 0
    
    # generate a random initial state that is outside of the asteroid
    pos = np.random.rand(3)
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