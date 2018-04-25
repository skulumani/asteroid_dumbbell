"""Plot the data from explore_main

Extended description of the module

Notes
-----
    This is an example of an indented section. It's like any other section,
    but the body is indented to help it stand out from surrounding text.

If a section is indented, then a section break is created by
resuming unindented text.

Attributes
----------
module_level_variable1 : int
    Descrption of the variable

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
import pdb
import numpy as np
import os
import h5py
import argparse

from point_cloud import wavefront
from visualization import graphics
import utilities

def exploration_generate_plots(data_path, img_path='/tmp/diss_explore', 
                               magnification=1, step=1):
    """Given a HDF5 file generated by explore (C++) this will generate some plots
    """

    if not os.path.exists(img_path):
        os.makedirs(img_path)

    with h5py.File(data_path, 'r') as hf:
        rv = hf['reconstructed_vertex']
        rw = hf['reconstructed_weight']

        # get all the keys
        v_keys = np.array(utilities.sorted_nicely(list(rv.keys())))
        w_keys = np.array(utilities.sorted_nicely(list(rw.keys())))

        v_initial = hf['initial_vertex'][()]
        f_initial = hf['initial_faces'][()]
        w_initial = np.squeeze(hf['initial_weight'][()])

        """Partial images during the reconstruction"""
        mfig = graphics.mayavi_figure(offscreen=True)
        mesh = graphics.mayavi_addMesh(mfig, v_initial, f_initial)
        ms = mesh.mlab_source
        graphics.mayavi_axes(mfig, [-1, 1, -1, 1, -1, 1], line_width=5, color=(1, 0, 0))
        graphics.mayavi_view(fig=mfig)

        partial_index = np.array([0, v_keys.shape[0]*1/4, v_keys.shape[0]*1/2,
                                  v_keys.shape[0]*3/4, v_keys.shape[0]*4/4-1],
                                 dtype=np.int)
        for img_index, vk in enumerate(partial_index):
            filename = os.path.join(img_path, 'partial_' + str(vk) + '.jpg')
            v = rv[str(vk)][()]
            # generate an image and save it 
            ms.reset(x=v[:, 0], y=v[:, 1], z=v[:,2], triangles=f_initial)
            graphics.mlab.savefig(filename, magnification=4)
        
        """Create a bunch of images for animation"""
        animation_path = os.path.join(img_path, 'animation')
        if not os.path.exists(animation_path):
            os.makedirs(animation_path)
        pdb.set_trace() 
        ms.reset(x=v_initial[:, 0], y=v_initial[:, 1], z=v_initial[:, 2], triangles=f_initial,
                 scalars=w_initial)
        graphics.mayavi_view(mfig)
        for ii, vk in enumerate(v_keys[::step]):
            filename = os.path.join(animation_path, str(ii).zfill(7) + '.jpg')
            v = rv[vk][()]
            w = np.squeeze(rw[str(vk)][()])
            ms.reset(x=v[:, 0], y=v[:, 1], z=v[:, 2], triangles=f_initial, scalars=w)
            graphics.mayavi_savefig(mfig, filename, magnification=magnification)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate plots from explore",
                                     formatter_class=argparse.RawTextHelpFormatter)

    # add options
    parser.add_argument("hdf5_file", help="The data file to read", type=str)
    parser.add_argument("img_path", help="The path to save images", type=str)

    args = parser.parse_args()

    exploration_generate_plots(args.hdf5_file, args.img_path);



