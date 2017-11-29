"""Use finite differencing to verify polyhedron acceleration
"""
import numpy as np
from dynamics import asteroid
from point_cloud import wavefront, polyhedron

delta = 1e-6

def finite_difference(pos, deltapos, ast):

    # calculate f(x + deltax) - f(x)
    U, dUdx, _, _ = ast.polyhedron_potential(pos)
    Udx, _, _, _ = ast.polyhedron_potential(pos + deltapos)
    # compare to gradient of f
    diff_U = Udx - U

    return diff_U, dUdx

def test_itokawa_obj():
    # pick a known position
    pos = np.random.uniform(1, 2, (3,))
    # apply a small perturbation
    deltapos = delta*np.random.uniform(0, 1, pos.shape)
    ast = asteroid.Asteroid('itokawa', 0, 'obj')
    diff_U, dUdx = finite_difference(pos, deltapos, ast)
    np.testing.assert_almost_equal(dUdx.dot(deltapos), diff_U)

def test_castalia_obj():
    # pick a known position
    pos = np.random.uniform(1, 2, (3,))
    # apply a small perturbation
    deltapos = delta*np.random.uniform(0, 1, pos.shape)
    ast = asteroid.Asteroid('castalia', 0, 'obj')
    diff_U, dUdx = finite_difference(pos, deltapos, ast)
    np.testing.assert_almost_equal(dUdx.dot(deltapos), diff_U)

def test_eros_obj():
    # pick a known position
    pos = np.random.uniform(1, 2, (3,))
    # apply a small perturbation
    deltapos = delta*np.random.uniform(0, 1, pos.shape)
    ast = asteroid.Asteroid('eros', 0, 'obj')
    diff_U, dUdx = finite_difference(pos, deltapos, ast)
    np.testing.assert_almost_equal(dUdx.dot(deltapos), diff_U)
