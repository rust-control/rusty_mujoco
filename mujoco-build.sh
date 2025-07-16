#!/bin/bash

./mujoco-patch.sh
cd ./src/mujoco-3.3.2
if [ -d build ]; then
    rm -rf build
else
    mkdir build
fi
cd build
cmake .. \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
    -DCMAKE_C_COMPILER=clang-14 -DCMAKE_CXX_COMPILER=clang++-14 -DMUJOCO_HARDEN=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
    -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_TEST_PYTHON_UTIL=OFF
cmake --build .
