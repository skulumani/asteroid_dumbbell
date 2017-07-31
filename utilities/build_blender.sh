#!/bin/bash

# this will clone and install the blender source and also build blender as a
# module

if [ -d "$HOME/blender-git" ]; then
    echo "Blender source directory already exits"
    cd "$HOME/blender-git/blender"
    git pull --rebase
else
    mkdir -p "$HOME/blender-git"
    cd "$HOME/blender-git"
    git clone https://git.blender.org/blender.git
    cd blender
    git submodule update --init --recursive
    git submodule foreach git checkout master
fi

# update submodules
git submodule foreach git pull --rebase origin master

sudo apt-get update
sudo apt-get install git build-essential cmake cmake-curses-gui

# install blender dependencies
cd "$HOME/blender-git"
/bin/bash "$HOME/blender-git/blender/build_files/build_environment/install_deps.sh"

# compile blender module for python from source
mkdir build
cd build
cmake ../blender \ 
    -DPYTHON_VERSION=3.5 \
    -DPYTHON_ROOT_DIR=$HOME/anaconda3/envs/asteroid \
    -DWITH_PYTHON_INSTALL=OFF \
    -DWITH_PLAYER=OFF \
    -DWITH_PYTHON_MODULE=ON \
    -DWITH_INSTALL_PORTABLE=ON \
    -DCMAKE_INSTALL_PREFIX=$HOME/anaconda3/envs/asteroid/lib/python3.5/site-packages

make -j8
make install


