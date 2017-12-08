#!/bin/bash

# setup script to install dependencies on Travis

sudo apt-get -qq update

echo "Downloading and Installing Miniconda"
# get miniconda installed
wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh -O $HOME/miniconda.sh
bash $HOME/miniconda.sh -b -p $HOME/anaconda3
export PATH="$HOME/anaconda3/bin:$PATH"
hash -r

echo "Configuring Conda settings"
# configure conda
conda config --set always_yes yes --set changeps1 no
conda update conda

echo "Downloading the shape models"
# download the shape model
wget https://github.com/skulumani/asteroid_dumbbell/releases/download/v0.3/shape_model.tar.gz -O ./data/shape_model/shape_model.tar.gz
tar xf ./data/shape_model/shape_model.tar.gz -C ./data/shape_model

echo "Creating the asteroid environment"
# setup development enviornment
conda env create --file ./utilities/asteroid_reduced.yml

echo "Activate asteroid environment"
source activate asteroid

echo "Setup is complete"
