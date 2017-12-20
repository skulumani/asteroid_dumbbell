"""Find the equilibria around an asteroid
"""
from dynamics import asteroid

import numpy as np
import scipy


def obj(pos, ast):
    """Objective function for minimization
    """
    omega = ast.omega
    scale = 1e6
    # compute potential at pos
    U, U_grad, U_grad_mat, Ulaplace = ast.polyhedron_potential(pos)

    # compute potential in rotating frame
    F = U_grad + np.array([omega**2 + pos[0], omega**2 * pos[1], 0])
    dF = U_grad_mat + omega**2 * np.diag([1, 1, 0])
    
    # scale both and return
    return F*scale, dF*scale

# initial guess of equilibrium point
x0 = np.array([1, 0, 0])
ast = asteroid.Asteroid('castalia', 0, 'obj')

# try fsolve

# newton iteration

# output
