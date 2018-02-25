#!/bin/bash

DIR="$(pwd)"
read -p "Enter the conda enviornment name" CONDA_ENV
# script to setup asteroid enviornment

# first check to ensure conda is installed

# create the enviornment

conda env create -n ${CONDA_ENV} -f ${DIR}/asteroid.yml

conda activate asteroid

conda clean --all

# build mayavi
bash ${DIR}/build_mayavi.sh

bash ${DIR}/build_cmake.sh
bash ${DIR}/build_boost.sh
bash ${DIR}/build_eigen.sh
bash ${DIR}/build_cgal.sh
# build blender

# build pcl

# build opencv

# build numba

# build cgal
