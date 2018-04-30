"""Cube

Gravitational potential of a cube, analytical

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""

import numpy as np

def w(x, y, z):
    """The w function inside the integrand
    """
    r = np.sqrt(x**2 + y**2 + z**2)

    w = x * y * np.log(z + r) - 1/2*x**2*np.arctan2(y*z, x*r)

    return w

def potential(point, axes=np.array([1, 1, 1])):
    """Compute the potential at the point for a cube with side lengths
    axes = (2a, 2b, 2c)
    """

    a = axes[0]/2
    b = axes[1]/2
    c = axes[2]/2

    x0 = point[0]
    y0 = point[1]
    z0 = point[2]

    G = 6.67408e-11 # km^3 / (kg * s ^2)
    rho = 1e6 # kg / km^3

    potential = G * rho *( w(a - x0, b - y0, c - z0) + w(b - y0, c - z0, a - x0) + w(c - z0, a - x0, b - y0) 
                          - w(-a - x0, -b - y0, -c - z0) - w(-b -y0, -c - z0, -a - x0) - w(-c - z0, -a - x0, -b - y0))

    return potential
