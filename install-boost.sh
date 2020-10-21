#!/bin/sh
set -ex

BOOST_VERSION=${1:-1.57.0}
BOOST_LIB=${2:-/usr/local}
BOOST_VERSION_UNDERSCORE=$( echo ${BOOST_VERSION} | tr '.' '_' )

cd /tmp
wget http://downloads.sourceforge.net/project/boost/boost/${BOOST_VERSION}/boost_${BOOST_VERSION_UNDERSCORE}.tar.gz
tar -xzf boost_${BOOST_VERSION_UNDERSCORE}.tar.gz
cd boost_${BOOST_VERSION_UNDERSCORE} && ./bootstrap.sh --prefix=${BOOST_LIB} && ./b2 install

# clean up
rm /tmp/boost_${BOOST_VERSION_UNDERSCORE}
