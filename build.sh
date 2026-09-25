#!/usr/bin/env bash
# Builds g2o -> stella_vslam -> stella_vslam_examples into ./local_install (no sudo).
#
# Prerequisites (one-time, needs sudo — not run by this script):
#   sudo apt update
#   sudo apt install -y build-essential pkg-config cmake git wget curl unzip \
#     libopencv-dev libeigen3-dev libyaml-cpp-dev libsuitesparse-dev libsqlite3-dev
#
# Usage:
#   git submodule update --init --recursive   # first time only
#   ./build.sh

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="$ROOT/local_install"
JOBS="$(nproc)"

mkdir -p "$PREFIX"

echo "==> [1/4] g2o"
cmake -S "$ROOT/g2o" -B "$ROOT/g2o/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DBUILD_SHARED_LIBS=ON \
  -DG2O_USE_CHOLMOD=OFF \
  -DG2O_USE_CSPARSE=ON \
  -DG2O_USE_OPENGL=OFF \
  -DG2O_USE_OPENMP=OFF \
  -DG2O_BUILD_EXAMPLES=OFF \
  -DG2O_BUILD_APPS=OFF
cmake --build "$ROOT/g2o/build" -j"$JOBS"
cmake --install "$ROOT/g2o/build"

echo "==> [2/4] stella_vslam"
cmake -S "$ROOT/stella_vslam" -B "$ROOT/stella_vslam/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$PREFIX" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DBOW_FRAMEWORK=FBoW \
  -DUSE_STACK_TRACE_LOGGER=OFF
cmake --build "$ROOT/stella_vslam/build" -j"$JOBS"
cmake --install "$ROOT/stella_vslam/build"

echo "==> [3/4] stella_vslam_examples (headless: no viewer)"
cmake -S "$ROOT/stella_vslam_examples" -B "$ROOT/stella_vslam_examples/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="$PREFIX" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DUSE_STACK_TRACE_LOGGER=OFF \
  -DUSE_GOOGLE_PERFTOOLS=OFF
cmake --build "$ROOT/stella_vslam_examples/build" -j"$JOBS"

echo "==> [4/4] ORB vocabulary file"
mkdir -p "$ROOT/vocab"
if [ ! -f "$ROOT/vocab/orb_vocab.fbow" ]; then
  curl -fL -o "$ROOT/vocab/orb_vocab.fbow" \
    https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
fi

echo "==> Done. Try:"
echo "    source $ROOT/env.sh"
echo "    \"\$RUN_VIDEO_SLAM\" -h"

# Notes on cmake_minimum_required and CMAKE_POLICY_VERSION_MINIMUM=3.5:
#   stella_vslam (and its nested submodule 3rd/tinycolormap) and stella_vslam_examples
#   all declare cmake_minimum_required(VERSION 3.1), which CMake >= 4.0 refuses outright
#   ("Compatibility with CMake < 3.5 has been removed"). This flag is CMake's
#   documented escape hatch; g2o didn't need it (its minimum is already 3.14).
#
# Note on g2oConfig.cmake: it used to unconditionally do find_dependency(OpenGL) even
# when built with G2O_USE_OPENGL=OFF, requiring a manual sed patch after install. Fixed
# at the source in g2o/cmake_modules/Config.cmake.in (now conditional on G2O_USE_OPENGL),
# so no patch step is needed here anymore.
#
# This script builds g2o/stella_vslam into local_install/ for the local core-algorithm
# testing tools (stella_vslam_examples, run_experiment.sh) — it does not need ROS2.
# For the ROS2 deployment stack (stella_vslam_ros, vslam_bringup), see colcon_build.sh
# instead, which builds g2o/stella_vslam a second time into colcon's own install/ prefix.
