#!/bin/sh
set -ex

GCC_VERSION=${1:-4.8}
G++_VERSION=${2:-4.8}

apt-get install -qq -y software-properties-common
add-apt-repository --yes ppa:ubuntu-toolchain-r/test
apt-get update && apt-get install -qq -y git wget nano vim cmake

# install compiler
apt-get update && apt-get install -qq -y gcc-4.8 g++-4.8
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 20
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 20

cd /tmp && wget https://raw.githubusercontent.com/wyq977/LBIBCell/add-travis/install-boost.sh
bash install-boost.sh; rm -rf install-boost.sh

# packages needed for vtk (OpenGL OSMesa)
apt-get install -qq -y freeglut3 freeglut3-dev mesa-common-dev mesa-utils libosmesa6 libxt-dev
# packages needed for vtk (according to group share)
apt-get install -qq -y libpng-dev libjpeg-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev mesa-common-dev mesa-utils-extra libgl1-mesa-dev libglapi-mesa doxygen

cd /tmp && wget https://raw.githubusercontent.com/wyq977/LBIBCell/add-travis/install-vtk.sh
bash install-vtk.sh; rm -rf install-vtk.sh