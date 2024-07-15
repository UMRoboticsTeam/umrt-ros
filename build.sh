#!/bin/bash

#Build Parameters
build_type=Release
install_type=symlink-install

if [[ -z "${COLCON_BUILD_EXECUTOR}" ]]; then
    COLCON_BUILD_EXECUTOR=sequential
fi

#Echo Build & Install Type
echo "Build type: $build_type, Install_type: $install_type, Executor: $COLCON_BUILD_EXECUTOR"

colcon build \
    --$install_type \
    --executor $COLCON_BUILD_EXECUTOR \
    --cmake-args -DCMAKE_BUILD_TYPE=$build_type \
    --cmake-args -DBUILD_TESTING=OFF \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    --cmake-args -DBUILD_SHARED_LIBS=ON

