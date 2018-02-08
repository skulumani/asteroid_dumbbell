#!/bin/bash

DIR="$(pwd)"
read -p "Enter the conda enviornment name" CONDA_ENV
# script to setup asteroid enviornment

# first check to ensure conda is installed

# create the enviornment

conda env create -n ${CONDA_ENV} -f ${DIR}/asteroid.yml

conda activate asteroid
conda uninstall --verbos mayavi vtk

conda clean --all
conda activate asteroid
pip install vtk
# build mayavi
bash ${DIR}/build_mayavi.sh

# build blender

# build pcl

# build opencv

# build numba

# build cgal
