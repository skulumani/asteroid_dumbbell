"""Add plot of high resolution area on castalia
"""
import pdb
import numpy as np
import os
import h5py
import argparse

from point_cloud import wavefront
from visualization import graphics
import utilities

view = (0, 25, 3.32, np.array([-0.04, -0.015, -0.029]))
# load the bumpy castalia
vb_true, fb_true = wavefront.read_obj('./data/shape_model/CASTALIA/castalia_bump.obj')
mfig = graphics.mayavi_figure(offscreen=False)
mesh = graphics.mayavi_addMesh(mfig, vb_true, fb_true)
graphics.mayavi_view(mfig, *view)
graphics.mayavi_savefig(mfig, '/tmp/castalia_bump_true.jpg', magnification=4)

# load the refinement final version
with h5py.File('./data/exploration/refine/20180619_castalia_refinement.hdf5', 'r') as hf:
    # oriignal vertices
    est_initial_vertices = hf['simulation_parameters/estimate_asteroid/initial_vertices'][()]
    num_vert = est_initial_vertices.shape[0]

    rv_group = hf['refinement/reconstructed_vertex']
    rf_group = hf['refinement/reconstructed_face']
    rw_group = hf['refinement/reconstructed_weight']
    Ra_group = hf['refinement/Ra']

    rv_keys = np.array(utilities.sorted_nicely(list(rv_group.keys())))

    v = rv_group[rv_keys[-1]][()]
    f = rf_group[rv_keys[-1]][()]
    w = rw_group[rv_keys[-1]][()]
    
    mfig_est = graphics.mayavi_figure(offscreen=False)
    mesh = graphics.mayavi_addMesh(mfig_est, v, f)
    graphics.mayavi_view(mfig_est, *view)
    graphics.mayavi_savefig(mfig_est, '/tmp/castalia_bump_est.jpg', magnification=4)
