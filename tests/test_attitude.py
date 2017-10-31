from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
import scipy.linalg
import kinematics.attitude as attitude


class TestHatAndVeeMap():
    x = np.random.rand(3)
    y = np.random.rand(3)

    def test_hat_map_zero(self):
        np.testing.assert_allclose(
            attitude.hat_map(self.x).dot(self.x), np.zeros(3))

    def test_hat_map_skew_symmetric(self):
        np.testing.assert_allclose(attitude.hat_map(
            self.x).T, -attitude.hat_map(self.x))

    def test_hat_vee_map_inverse(self):
        np.testing.assert_allclose(attitude.vee_map(
            attitude.hat_map(self.x)), self.x)

    def test_hat_map_cross_product(self):
        np.testing.assert_allclose(attitude.hat_map(
            self.x).dot(self.y), np.cross(self.x, self.y))
        np.testing.assert_allclose(attitude.hat_map(self.x).dot(
            self.y), -attitude.hat_map(self.y).dot(self.x))


class TestEulerRot():
    angle = (2 * np.pi - 0) * np.random.rand(1) + 0

    def test_rot1_orthogonal(self):
        mat = attitude.rot1(self.angle)
        np.testing.assert_array_almost_equal(mat.T.dot(mat), np.eye(3, 3))

    def test_rot2_orthogonal(self):
        mat = attitude.rot2(self.angle)
        np.testing.assert_array_almost_equal(mat.T.dot(mat), np.eye(3, 3))

    def test_rot3_orthogonal(self):
        mat = attitude.rot3(self.angle)
        np.testing.assert_array_almost_equal(mat.T.dot(mat), np.eye(3, 3))

    def test_rot1_determinant_1(self):
        mat = attitude.rot1(self.angle)
        np.testing.assert_allclose(np.linalg.det(mat), 1)

    def test_rot2_determinant_1(self):
        mat = attitude.rot2(self.angle)
        np.testing.assert_allclose(np.linalg.det(mat), 1)

    def test_rot3_determinant_1(self):
        mat = attitude.rot3(self.angle)
        np.testing.assert_allclose(np.linalg.det(mat), 1)


class TestExponentialMap():
    angle = (2 * np.pi - 0) * np.random.rand(1) + 0
    t = (100 - 0) * np.random.rand(1) + 0
    alpha = np.random.rand(1)

    axis = np.random.rand(3)
    axis = axis / np.linalg.norm(axis)

    def test_expm_rot1_equivalent(self):
        R = scipy.linalg.expm(
            self.angle * attitude.hat_map(np.array([1, 0, 0])))
        R1 = attitude.rot1(self.angle)
        np.testing.assert_almost_equal(R.T.dot(R1), np.eye(3, 3))

    def test_expm_rot2_equivalent(self):
        R = scipy.linalg.expm(
            self.angle * attitude.hat_map(np.array([0, 1, 0])))
        R2 = attitude.rot2(self.angle)
        np.testing.assert_almost_equal(R.T.dot(R2), np.eye(3, 3))

    def test_expm_rot3_equivalent(self):
        R = scipy.linalg.expm(
            self.angle * attitude.hat_map(np.array([0, 0, 1])))
        R3 = attitude.rot3(self.angle)
        np.testing.assert_almost_equal(R.T.dot(R3), np.eye(3, 3))

    def test_expm_rot1_derivative(self):
        axis = np.array([1, 0, 0])
        R_dot = self.alpha * attitude.hat_map(axis).dot(
            scipy.linalg.expm(self.alpha * self.t * attitude.hat_map(axis)))
        R1_dot = attitude.rot1(
            self.alpha * self.t).dot(attitude.hat_map(self.alpha * axis))
        np.testing.assert_almost_equal(R_dot, R1_dot)

    def test_expm_rot2_derivative(self):
        axis = np.array([0, 1, 0])
        R_dot = self.alpha * attitude.hat_map(axis).dot(
            scipy.linalg.expm(self.alpha * self.t * attitude.hat_map(axis)))
        R2_dot = attitude.rot2(
            self.alpha * self.t).dot(attitude.hat_map(self.alpha * axis))
        np.testing.assert_almost_equal(R_dot, R2_dot)

    def test_expm_rot3_derivative(self):
        axis = np.array([0, 0, 1])
        R_dot = self.alpha * attitude.hat_map(axis).dot(
            scipy.linalg.expm(self.alpha * self.t * attitude.hat_map(axis)))
        R3_dot = attitude.rot3(
            self.alpha * self.t).dot(attitude.hat_map(self.alpha * axis))
        np.testing.assert_almost_equal(R_dot, R3_dot)
