#!/bin/bash

#Build Parameters
build_type=Release
install_type=symlink-install

#Echo Build & Install Type
echo "Build type: $build_type, Install_type: $install_type"

#Build In Sequential
echo "Sequential Build" && \
MAKEFLAGS="-j1 -l1" colcon build \
    --$install_type \
    --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=$build_type \
    --cmake-args -DBUILD_TESTING=OFF \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    --cmake-args -DBUILD_SHARED_LIBS=ON

