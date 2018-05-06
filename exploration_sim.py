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

from lib import asteroid, surface_mesh, cgal, controller, mesh_data, reconstruct
from lib import surface_mesh

from dynamics import dumbbell, eoms, controller
from point_cloud import wavefront
from kinematics import attitude

def initialize():
    """Initialize all the things for the simulation
    """
    logger = logging.getLogger(__name__)
    logger.info('Initialize asteroid and dumbbell objects')

    AbsTol = 1e-9
    RelTol = 1e-9
    
    # true asteroid and dumbbell
    v, f = wavefront.read_obj('./data/shape_model/CASTALIA/castalia.obj')

    true_ast_meshdata = mesh_data.MeshData(v, f)
    true_ast_meshparam = asteroid.MeshParam(true_ast)
    true_ast = asteroid.Asteroid('castalia', ast_param)
    dum = dumbbell.Dumbbell(m1=500, m2=500, l=0.003)
    
    # estimated asteroid (starting as an ellipse)
    ellipsoid = surface_mesh.SurfMesh(ast.get_axes[0], ast.get_axes[1], ast.get_axes[2])

    est_ast_meshdata = mesh_data.MeshData(ellipsoid.get_verts(), ellipsoid.get_faces())
    est_ast_rmesh = reconstruct.ReconstructMesh(est_ast_meshdata)

    # controller functions 
    complete_controller = controller.Controller()
    
    # lidar object

    # raycaster from c++
    caster = cgal.RayCaster(true_ast_meshdata) 

    return ast, dum, complete_controller, AbsTol, RelTol

def simulate():
    """Actually run the simulation around the asteroid
    """
    logger = logging.getLogger(__name__)

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
    
    # initialize the simulation
    ast, dum, complete_controller, AbsTol, RelTol = initalize()

if __name__ == "__main__":
    logging_file = tempfile.mkstemp(suffix='.txt.')[1]
    output_path = tempfile.mkdtemp()

    logging.basicConfig(filename=logging_file,
                        filemode='w', level=logging.INFO,
                        format='%(asctime)s %(levelname)-8s %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    print("Logging to {}".format(logging_file))

    parser = argparse.ArgumeentParser(description="Exploration and asteroid reconstruction simulation",
                                      formatter_class=argparse.RawTextHelpFormatter)
    
    parser.add_argument("-o", "--reconstruct_data",
                        help="Filename to store the reconstruction data")
    


