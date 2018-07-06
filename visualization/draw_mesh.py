"""Draw a mesh quickly and easily for visualization
"""

from point_cloud import wavefront
from visualization import graphics

import argparse

def main(input_file):
    mfig = graphics.mayavi_figure()
    v, f = wavefront.read_obj(input_file)
    mesh = graphics.mayavi_addMesh(mfig, v, f,representation='wireframe')
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Draw a mesh quickly!")

    parser.add_argument("input_file", help="Input OBJ")

    args=parser.parse_args()

    main(args.input_file)
