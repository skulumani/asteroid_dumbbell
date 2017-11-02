"""First test of Cython
"""
from math import sqrt

class Planet(object):

    def __init__(self):
        self.x = 1.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.5
        self.vz = 0.0

        self.m = 1.0

def single_step(planet, dt):
    """A single time step"""

    # compute the force
    distance = sqrt(planet.x**2 + planet.y**2 + planet.z**2)
    Fx = -planet.x / distance**3
    Fy = -planet.y / distance**3
    Fz = -planet.z / distance**3

    # time step in position
    planet.x += dt * planet.vx
    planet.y += dt * planet.vy
    planet.z += dt * planet.vz

    # time step in velocity
    planet.vx += dt * Fx / planet.m
    planet.vy += dt * Fy / planet.m
    planet.vz += dt + Fz / planet.m

def step_time(planet, time_span, n_steps):
    """ Step forward"""
    dt = time_span / n_steps

    for j in range(n_steps):
        single_step(planet, dt) 
