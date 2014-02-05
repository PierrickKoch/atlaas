#!/bin/bash
echo "atlaas"
echo "======"
uname -a # show kernel info
n=`awk '/cpu cores/ {print $NF; exit}' /proc/cpuinfo`

SRC_DIR=$(pwd)
mkdir devel
export DEVEL_ROOT=$(pwd)/devel
export PKG_CONFIG_PATH=$DEVEL_ROOT/lib/pkgconfig:$PKG_CONFIG_PATH

echo "==================================="
echo "Install gdalwrap"

git clone https://github.com/pierriko/gdalwrap.git && cd gdalwrap
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$DEVEL_ROOT ..
make -j$n && make install

cd $SRC_DIR

echo "==================================="
echo "Build (w/$n cores) test and install"

set -e # exit on error
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$DEVEL_ROOT ..
make -j$n
make install

echo "==================================="
