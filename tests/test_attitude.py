import numpy as np
import attitude_ref.attitude as attitude

class TestHatAndVeeMap():
    x = np.random.rand(3)
    y = np.random.rand(3)

    def test_hat_map_zero(self):
        np.testing.assert_allclose(attitude.hat_map(self.x).dot(self.x),np.zeros(3))

    def test_hat_map_skew_symmetric(self):
        np.testing.assert_allclose(attitude.hat_map(self.x).T, -attitude.hat_map(self.x))

    def test_hat_vee_map_inverse(self):
        np.testing.assert_allclose(attitude.vee_map(attitude.hat_map(self.x)), self.x)

    def test_hat_map_cross_product(self):
        np.testing.assert_allclose(attitude.hat_map(self.x).dot(self.y), np.cross(self.x, self.y))
        np.testing.assert_allclose(attitude.hat_map(self.x).dot(self.y), -attitude.hat_map(self.y).dot(self.x))      