"""Generate plots of asteroids

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
from point_cloud import wavefront
from visualization import graphics

import os
import argparse

def asteroid_plots(asteroid_name, asteroid_path, output_directory):
    """This will generate 4 images for each asteroid.

    A view along each axis followed by a orthographic view
    """
    
    # load the asteroid
    v, f = wavefront.read_obj(asteroid_path)
    # create the image
    mfig = graphics.mayavi_figure(offscreen=True)
    graphics.mayavi_addMesh(mfig, v, f)
    filename = os.path.join(output_directory, asteroid_name + '_isometric.jpg')
    graphics.mayavi_savefig(mfig, filename, magnification=4)
    
    graphics.mayavi_view(mfig, azimuth=0, elevation=0)
    filename = os.path.join(output_directory, asteroid_name + '_x.jpg')
    graphics.mayavi_savefig(mfig, filename, magnification=4)

    graphics.mayavi_view(mfig, azimuth=90, elevation=0)
    filename = os.path.join(output_directory, asteroid_name + '_y.jpg')
    graphics.mayavi_savefig(mfig, filename, magnification=4)

    graphics.mayavi_view(mfig, azimuth=0, elevation=90)
    filename = os.path.join(output_directory, asteroid_name + '_z.jpg')
    graphics.mayavi_savefig(mfig, filename, magnification=4)

if __name__ == "__main__": 
    parser = argparse.ArgumentParser(description="Generate views of an asteroid")

    parser.add_argument("asteroid_name", help="Name of asteroid", type=str, nargs=1)
    parser.add_argument("asteroid_path", help="Path to OBJ file", type=str, nargs=1)
    parser.add_argument("output_path", help="Path to save jpg images", type=str, nargs=1)

    args = parser.parse_args()

    asteroid_plots(args.asteroid_name[0], args.asteroid_path[0], args.output_path[0])
