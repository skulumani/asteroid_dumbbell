import numpy as np
import attitude_ref.attitude as attitude


x = np.random.rand(3,1)

def test_hat_map_skew_symmetric(self):
    np.testing.assert_allclose(attitude.hat_map(self.x)*self.x,np.zeros(3))
