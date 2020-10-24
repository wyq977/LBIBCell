#!/bin/sh
set -ex

VTK_VERSION=${1:-5.10.1}
VTK_LIB=${2:-/usr/local}

# clone
git clone --single-branch --branch v$VTK_VERSION https://gitlab.kitware.com/vtk/vtk.git /tmp/vtk

# out-of-source build
# https://stackoverflow.com/questions/28761702/getting-error-glintptr-has-not-been-declared-when-building-vtk-on-linux
# Not sure if these two options are needed
# -D VTK_USE_RENDERING:BOOL=OFF \
# -D VTK_USE_DISPLAY:BOOL=OFF \
mkdir /tmp/vtk-build && cd /tmp/vtk-build
cmake -Wno-dev \
-D CMAKE_BUILD_TYPE:STRING=Release \
-D BUILD_SHARED_LIBS:BOOL=ON \
-D CMAKE_INSTALL_PREFIX:STRING=$VTK_LIB \
-D CMAKE_C_FLAGS:STRING=-DGLX_GLXEXT_LEGACY \
-D CMAKE_CXX_FLAGS:STRING=-DGLX_GLXEXT_LEGACY \
/tmp/vtk

# build
make --silent -j $(cat /proc/cpuinfo | grep processor | wc -l) VERBOSE=1 && \
make install --silent

# clean up
rm -rf /tmp/vtk && rm -rf /tmp/vtk-build
