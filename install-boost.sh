#!/bin/sh
set -ex

cd /tmp
wget http://sourceforge.net/projects/boost/files/boost/1.57.0/boost_1_57_0.tar.gz
tar -xzvf boost_1_57_0.tar.gz
cd boost_1_57_0 && ./bootstrap.sh --prefix=/usr/local && ./b2 install

# clean up
cd /tmp && rm boost_1_57_0
