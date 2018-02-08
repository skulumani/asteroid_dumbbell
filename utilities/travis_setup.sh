#!/bin/bash

# setup script to install dependencies on Travis
# If it does exist then only update the conda environment rather than create it
sudo apt-get -qq update

# Script to download and install mayavi from source
MAYAVI_REPO="https://github.com/enthought/mayavi.git"

ANACONDA_ENV="asteroid"
ANACONDA_PATH="$HOME/anaconda3/envs"

echo "Downloading and Installing Miniconda"
# get miniconda installed
# check if anaconda3 directory exists
if [ -f "$HOME/anaconda3/bin/conda" ]; then
    echo "Anaconda already installed"
else
    echo "Anaconda not installed"
    wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh -O $HOME/miniconda.sh
    bash $HOME/miniconda.sh -b -u -p $HOME/anaconda3
fi

export PATH="$HOME/anaconda3/bin:$PATH"
hash -r

echo "Configuring Conda settings"
# configure conda
conda config --set always_yes yes --set changeps1 no
conda update conda
echo ". /home/travis/anaconda3/etc/profile.d/conda.sh" >> ~/.bashrc
export PATH="$HOME/anaconda3/bin:$PATH"
hash -r 
source $HOME/anaconda3/bin/activate

echo "Downloading the shape models"
# download the shape model
wget https://github.com/skulumani/asteroid_dumbbell/releases/download/v0.3/shape_model.tar.gz -O ./data/shape_model/shape_model.tar.gz
tar xf ./data/shape_model/shape_model.tar.gz -C ./data/shape_model

echo "Creating the asteroid environment"
# setup development enviornment
if [ -d "$HOME/anaconda3/envs/asteroid" ]; then
    echo "asteroid enviornment exists. Just update"
    conda env update --name asteroid --file ./utilities/asteroid.yml
else
    echo "No asteroid enviornment"
    conda env create --name asteroid --face ${DIR}/asteroid.yml

    conda activate asteroid
    conda uninstall --verbose mayavi vtk

    conda clean --all
    conda activate asteroid
    pip install vtk
    # build mayavi
    bash ./utilities/build_mayavi.sh
fi


echo "Make sure you've installed the conda environment!!!"
# read -p "Enter the conda enviornment to install mayavi: " ANACONDA_ENV

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

conda activate ${ANACONDA_ENV}

python setup.py install

echo "Mayavi installed"

echo "Setup is complete"
