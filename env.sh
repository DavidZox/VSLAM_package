#!/usr/bin/env bash
# source this file before running any stella_vslam_examples executable:
#   source /home/david/VSLAM_package/env.sh
export VSLAM_ROOT="/home/david/VSLAM_package"
export LD_LIBRARY_PATH="$VSLAM_ROOT/local_install/lib:$LD_LIBRARY_PATH"
export VOCAB_FILE="$VSLAM_ROOT/vocab/orb_vocab.fbow"
export RUN_VIDEO_SLAM="$VSLAM_ROOT/stella_vslam_examples/build/run_video_slam"
