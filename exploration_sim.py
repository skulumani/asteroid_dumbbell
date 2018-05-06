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
import logging
import os
import tempfile
import argparse

import h5py
import numpy as np
from scipy import integrate

from lib import asteroid, surface_mesh

from dynamics import dumbbell, eoms, controller
from point_cloud import wavefront
from kinematics import attitude

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

def simulate():
    """Actually run the simulation around the asteroid
    """
    logger = logging.getLogger(__name__)

    # initialize the simulation
    ast, dum, complete_controller, AbsTol, RelTol = initalize()

    num_steps = int(1e3)
    time = np.linspace(0, num_steps, num_steps)
    t0, tf = time[0], time[-1]
    dt = time[1] - time[0]

    # define the initial condition
    initial_pos = np.array([1.5, 0, 0])
    initial_vel = np.array([0, 0, 0])
    initial_R = attitude.rot3(np.pi / 2).reshape(-1)
    initial_w = np.array([0, 0, 0])
    initial_state = np.hstack((initial_pos, initial_vel, initial_R, initial_w))

if __name__ == "__main__":
    logging_file = tempfile.mkstemp(suffix='.txt.')[1]
    output_path = tempfile.mkdtemp()

    logging.basicConfig(filename=logging_file,
                        filemode='w', level=logging.INFO,
                        format='%(asctime)s %(levelname)-8s %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    print("Logging to {}".format(logging_file))


