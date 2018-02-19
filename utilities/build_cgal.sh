#!/bin/bash

CGAL_VER='CGAL-4.11'
INSTALL_DIR="$HOME/$CGAL_VER"

CGAL_RELEASE_URL='https://github.com/CGAL/cgal/releases/download/releases%2F'${CGAL_VER}'/'${CGAL_VER}'.tar.xz'

# check if the temp dir was created
if [[ ! "$INSTALL_DIR" || ! -d "$INSTALL_DIR" ]]; then
    echo "Creating CGAL Dir"
    mkdir $INSTALL_DIR
else
    mv $INSTALL_DIR /tmp/CGAL
    mkdir $INSTALL_DIR
fi

# # delete the temp directory on cleanup
# function cleanup {
#     rm -rf "$TEMP_DIR"
#     echo "Deleted temp working directory $TEMP_DIR"
# }

# trap cleanup EXIT

echo "Installing some dependencies"
sudo apt-get update
sudo apt-get install cmake build-essential libboost-dev libgmp-dev libmpfr-dev zlib1g-dev
sudo apt-get install libgl1 libstdc++6 libgcc1 libc6 libntl-dev libeigen3-dev 
# install CGAL for Python and build it from source for C++
echo "Verify you're using the correct conda environment"

read -p "Enter to install CGAL"

# conda install -c conda-forge cgal

# download the source tarball
cd ${INSTALL_DIR}
wget ${CGAL_RELEASE_URL} $INSTALL_DIR
tar xf ${CGAL_VER}.tar.* 

cd $INSTALL_DIR/$CGAL_VER
cmake -DWITH_examples=ON -DWITH_demos=ON -DWITH_CGAL_Qt5=OFF .
make 
# make examples
# make demos
sudo make install
