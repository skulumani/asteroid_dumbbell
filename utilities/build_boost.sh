#!/bin/bash
BOOST_VER=1.66.0
BOOST_SHA256_SUM="bd0df411efd9a585e5a2212275f8762079fed8842264954675a4fddc46cfcf60"
BOOST_URL="https://dl.bintray.com/boostorg/release/${BOOST_VER}/source/boost_1_66_0.tar.gz"
TEMP_DIR="$(mktemp -d)"
INSTALL_DIR="/usr/local"

# This will install the latest Boost and the Boost Python library
echo "We're going to download and install the latest Boost"
read -p "Press Enter to continue"

if [[ ! "$TEMP_DIR" || ! -d "$TEMP_DIR" ]]; then
    echo "Could not create temp dir"
    exit 1
fi

# delete the temp directory on cleanup
function cleanup {
    rm -rf "$TEMP_DIR"
    echo "Deleted temp working directory $TEMP_DIR"
}

trap cleanup EXIT

echo "Installing some dependencies"
sudo apt-get install -y build-essential g++ libicu-dev libbz2-dev autotools-dev

echo "Now downloading Boost"
cd ${TEMP_DIR}
mkdir boost
wget ${BOOST_URL} -O ${TEMP_DIR}/boost.tar.gz
# verify sha256 sum
if ! echo "${BOOST_SHA256_SUM} boost.tar.gz" | sha256sum -c; then
    echo "Checksum does not match. Aborting!!"
    exit 1
fi

tar -xzf boost.tar.gz -C ./boost --strip-components=1

# copy to /usr/local/include
# echo "Now copying to ${INSTALL_DIR}/boost"
# sudo mv boost/boost ${INSTALL_DIR}

echo "Now installing Boost and compiled libraries"
cd boost
./bootstrap.sh --prefix=${INSTALL_DIR} --with-libraries=all --with-python=$HOME/anaconda3/bin/python3

sudo ./b2 -j 4 install

echo "Boost and Boost-Python are installed to $INSTALL_DIR"
read -p "Press Enter to exit"
