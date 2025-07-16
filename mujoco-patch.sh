#!/bin/bash

# Based on https://github.com/google-deepmind/mujoco/pull/2693,
# enabling to build MuJoCo as a static library

MUJOCO_DIR="./mujoco-3.3.2"

sed -r -i 's/^add_library\(mujoco SHARED \$\{MUJOCO_RESOURCE_FILES\}\)$/add_library(mujoco ${MUJOCO_RESOURCE_FILES})/' "$MUJOCO_DIR/CMakeLists.txt"
sed -r -i 's/^target_compile_definitions\(mujoco PRIVATE _GNU_SOURCE CCD_STATIC_DEFINE MUJOCO_DLL_EXPORTS -DMC_IMPLEM_ENABLE\)$/target_compile_definitions(mujoco PRIVATE _GNU_SOURCE CCD_STATIC_DEFINE MJ_STATIC -DMC_IMPLEM_ENABLE)/' "$MUJOCO_DIR/CMakeLists.txt"
sed -r -i 's/TARGETS mujoco$/TARGETS mujoco lodepng/' "$MUJOCO_DIR/CMakeLists.txt"
sed -r -i 's/target_include_directories\(lodepng PUBLIC \$\{lodepng_SOURCE_DIR\}\)$/target_include_directories(lodepng PUBLIC $<BUILD_INTERFACE:${lodepng_SOURCE_DIR}>)/' "$MUJOCO_DIR/cmake/MujocoDependencies.cmake"
