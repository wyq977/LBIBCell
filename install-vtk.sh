#!/bin/sh
set -ex

cd /tmp
git clone --single-branch --branch v5.10.1 https://gitlab.kitware.com/vtk/vtk.git && \

# out-of-source build
mkdir /tmp/vtk-build && cd /tmp/vtk-build
cmake -Wno-dev \
-D CMAKE_BUILD_TYPE:STRING=Release \
-D BUILD_SHARED_LIBS:BOOL=ON \
-D CMAKE_INSTALL_PREFIX:STRING=/usr/lib/vtk \
-D CMAKE_C_FLAGS:STRING=-DGLX_GLXEXT_LEGACY \
-D CMAKE_CXX_FLAGS:STRING=-DGLX_GLXEXT_LEGACY \
/tmp/vtk

# build
make --silent -j $(cat /proc/cpuinfo | grep processor | wc -l) VERBOSE=1 && \
make install --silent

# clean up
cd /tmp && rm -rf vtk && rm -rf vtk-build
