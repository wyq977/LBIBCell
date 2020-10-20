#!/bin/sh
set -ex

cd /tmp
wget -c http://sourceforge.net/projects/boost/files/boost/1.57.0/boost_1_57_0.tar.gz -O - | sudo tar -xz
cd boost_1_57_0 && ./bootstrap.sh --prefix=/usr/local && ./b2 install

# clean up
cd /tmp && rm boost_1_57_0
