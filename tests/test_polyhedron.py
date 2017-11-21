"""Test polyhedron potential functions
"""
import numpy as np
from dynamics import asteroid
from wavefront import polyhedron

class TestFaceContributionItokawaMat():
    # define asteroid
    ast = asteroid.Asteroid('itokawa', 32, 'mat')
    pos = np.random.uniform(1, 2, size=(3,))
    V = ast.V
    num_v = V.shape[0]

    r_v = V - np.tile(pos, (num_v, 1))
    Fa, Fb, Fc = ast.asteroid_grav['Fa'], ast.asteroid_grav['Fb'], ast.asteroid_grav['Fc']
    F_face = ast.asteroid_grav['F_face']
    w_face = polyhedron.laplacian_factor(r_v, Fa, Fb, Fc)
    # call both functions
    U_face_loop, U_grad_face_loop, U_grad_mat_face_loop = polyhedron.face_contribution_loop(r_v, Fa, F_face, w_face)
    # make sure things match
