import dynamics.dumbbell as dumbbell
import dynamics.asteroid as asteroid
import attitude_ref.attitude as attitude

import numpy as np

class TestDumbbell():

    ast = asteroid.Asteroid('castalia',32)
    dum = dumbbell.Dumbbell()
    angle = (2*np.pi- 0) * np.random.rand(1) + 0
    
    # generate a random initial state that is outside of the asteroid
    pos = np.random.rand
    R = attitude.rot1(angle)

    def test_inertia_symmetric(self):
        np.testing.assert_array_almost_equal(self.dum.J.T, self.dum.J)

    def test_eoms_inertial(self):
