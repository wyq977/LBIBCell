#!/bin/sh
set -ex

VTK_VERSION=${1:-5.10.1}
VTK_LIB=${2:-/usr/local/lib}

echo ${VTK_LIB}
echo ${VTK_VERSION}
