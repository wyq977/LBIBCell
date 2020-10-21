#!/bin/sh
set -ex

GCC_VERSION=${1:-4.8}
G++_VERSION=${2:-4.8}

apt-get install -qq -y software-properties-common
add-apt-repository --yes ppa:ubuntu-toolchain-r/test
apt-get update && apt-get install -qq -y git wget
apt-get update && apt-get install -qq -y cmake

apt-get update && apt-get install -qq -y gcc-4.8 g++-4.8
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 20
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 20

cd /tmp && wget https://raw.githubusercontent.com/wyq977/LBIBCell/add-travis/install-boost.sh
bash install-boost.sh; rm -rf install-boost.sh

# OpenGL OSMesa
apt-get update && apt-get install -qq -y freeglut3 freeglut3-dev mesa-common-dev mesa-utils libosmesa6