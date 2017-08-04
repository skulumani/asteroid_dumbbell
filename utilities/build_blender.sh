#!/bin/bash

# this will clone and install the blender source and also build blender as a
# module

echo "Make sure you've installed the asteroid conda environment!!!"
read -p "Press Enter to continue"

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

# now build blender
echo "Now we're going to build blender"
read -p "Press Enter to continue"

# compile blender module for python from source
mkdir build
cd build
cmake ../blender -DCMAKE_INSTALL_PREFIX=$HOME/anaconda3/envs/asteroid/lib/python3.5/site-packages \
    -DPYTHON_VERSION=3.5 \
    -DPYTHON_ROOT_DIR=$HOME/anaconda3/envs/asteroid \
    -DWITH_PYTHON_INSTALL=OFF \
    -DWITH_PLAYER=OFF \
    -DWITH_PYTHON_MODULE=ON \
    -DWITH_INSTALL_PORTABLE=ON \

make -j8

echo "Now we're going to install Blender Python module"
read -p "Press enter to continue"

make install

echo "All done!"



