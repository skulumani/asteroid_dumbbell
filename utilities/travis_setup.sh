#!/bin/bash

# setup script to install dependencies on Travis
# TODO Setup travis cache and have logic to check if $HOME/anaconda3 already exists
# If it does exist then only update the conda environment rather than create it
sudo apt-get -qq update

echo "Downloading and Installing Miniconda"
# get miniconda installed
# check if anaconda3 directory exists
if [ -f "$HOME/anaconda3/bin/conda" ]; then
    echo "Anaconda already installed"
else
    echo "Anaconda not installed"
    wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh -O $HOME/miniconda.sh
    bash $HOME/miniconda.sh -b -p $HOME/anaconda3
fi

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
if [ -d "$HOME/anaconda3/envs/asteroid" ]; then
    echo "asteroid enviornment exists. Just update"
    conda env update -n asteroid ./utilities/asteroid_reduced.yml
else
    echo "No asteroid enviornment"
    conda env create --file ./utilities/asteroid_reduced.yml
fi

echo "Setup is complete"
