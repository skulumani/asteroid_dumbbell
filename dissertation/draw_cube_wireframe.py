"""Draw a mesh quickly and easily for visualization
"""
import numpy as np
from point_cloud import wavefront
from visualization import graphics

import os
import argparse

def main(input_file):
    mfig = graphics.mayavi_figure()
    v, f = wavefront.read_obj(input_file)
    mesh = graphics.mayavi_addMesh(mfig, v, f,color=(1, 0, 0), representation='wireframe')
    graphics.mayavi_addMesh(mfig, v, f,representation='surface')
    graphics.mlab.view(*(0, 90, 2.76, np.array([0, 0, 0]))) 
    print(input_file.split('.'))
    graphics.mayavi_savefig(mfig, '/tmp/isotropic.jpg', magnification=4)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Draw a mesh quickly!")

    parser.add_argument("input_file", help="Input OBJ")

    args=parser.parse_args()

    main(args.input_file)
