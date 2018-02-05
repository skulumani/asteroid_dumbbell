#!/bin/bash

set -x
set -e
# Script to download and install mayavi from source
MAYAVI_REPO="https://github.com/enthought/mayavi.git"

ANACONDA_PATH="/home/shankar/anaconda3/envs"
ANACONDA_ENV="asteroid"  # change to the specific anaconda enviornment you want

echo "Make sure you've installed the asteroid conda environment!!!"
read -p "Press Enter to continue"

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

source activate ${ANACONDA_ENV}

python setup.py install

echo "Mayavi installed"
