#!/bin/bash

# this will download and build opencv and install it into a Python environment
# Make sure you create the asteroid conda enviornment first before running this script
OPENCV_VERSION=3.2.0

echo "This will download and build OpenCV"
read -p "Press ENTER to continue"

if [ -d "$HOME/opencv" ]; then
    echo "OpenCV source already exists"
    cd "$HOME/opencv"
    git checkout $OPENCV_VERSION
else
    mkdir -p "$HOME/opencv"
    cd "$HOME/opencv"
    git clone https://github.com/opencv/opencv.git .
    git checkout $OPENCV_VERSION
fi

if [ -d "$HOME/opencv_contrib" ]; then
    echo "OpenCV Contrib source already exists"
    cd "$HOME/opencv_contrib"
    git checkout $OPENCV_VERSION
else
    mkdir -p "$HOME/opencv_contrib"
    cd "$HOME/opencv_contrib"
    git clone https://github.com/opencv/opencv_contrib.git .
    git checkout $OPENCV_VERSION
fi

# update build requirements
sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get -y install \
    build-essential \
    cmake \
    pkg-config \
    libjpeg8-dev \
    libtiff5-dev \
    libjasper-dev \
    libpng12-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libdc1394-22-dev \
    libxine2-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran python2.7-dev python3.5-dev 

# now build opencv
cd "$HOME/opencv"
mkdir build
cd build

echo "This might install it in a weird location"
echo "Make sure cv2.so is in $HOME/anaconda/envs/asteroid/lib/python3.5/site-packages"
read -p "Press enter to continue"

# call cmake
cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=$HOME/anaconda3/envs/asteroid \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=$HOME/opencv_contrib/modules \
    -D PYTHON_EXECUTABLE=$HOME/anaconda3/envs/asteroid/bin/python \
    -D BUILD_EXAMPLES=ON ..

echo "Check and make sure Python 3 paths are correct"
read -p "Press Enter to continue"

make -j8
make install

