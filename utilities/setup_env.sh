#!/bin/bash

DIR="$(pwd)"
# script to setup asteroid enviornment

# first check to ensure conda is installed

# create the enviornment

conda env create -f ${DIR}/asteroid.yml

# build mayavi
bash ${DIR}/build_mayavi.sh

# build blender

# build pcl

# build opencv

# build numba

# build cgal
