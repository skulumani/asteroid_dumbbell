"""Exploration Simulation

Simulation of a spacecraft exploring an asteroid

States are chosen to minimize a cost function tied to the uncertainty of the 
shape. Full SE(3) dynamics and the polyhedron potential model is used

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import pdb
import h5py
import numpy as np
from scipy import integrate

from lib import asteroid, surface_mesh

from dynamic import dummbell, eoms, controller
from point_cloud import wavefront

def initialize():
    """Initialize all the things for the simulation
    """
    logger = logging.getLogger(__name__)
    logger.info('Initialize asteroid and dumbbell objects')
    
    ast_param = asteroid.MeshParam(v, f)
    ast = asteroid.Asteroid('castalia', ast_param)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)

    # controller functions 
    complete_controller = controller.Controller()
    AbsTol = 1e-9
    RelTol = 1e-9

    return ast, dum, complete_controller, AbsTol, RelTol

