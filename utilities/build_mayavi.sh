#!/bin/bash

set -x
set -e
# Script to download and install mayavi from source
MAYAVI_REPO="https://github.com/enthought/mayavi.git"

ANACONDA_PATH="/home/shankar/anaconda3/envs"

echo "Make sure you've installed the conda environment!!!"
# read -p "Enter the conda enviornment to install mayavi: " ANACONDA_ENV
ANACONDA_ENV="asteroid"

# clone mayavi repo
if [ -d "/tmp/mayavi" ]; then
    echo "Mayavi source directory already exits"
    cd "/tmp/mayavi"
    git checkout master
else
    mkdir -p "/tmp/mayavi"
    cd "/tmp/mayavi"
    git clone ${MAYAVI_REPO} .
    git checkout master
fi
conda activate base
conda activate ${ANACONDA_ENV}

python setup.py install

echo "Mayavi installed"
