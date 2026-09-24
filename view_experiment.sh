#!/usr/bin/env bash
# 用 PangolinViewer 開視窗實際看某份 config 跑起來的樣子(特徵點 + 3D 地圖即時建構)。
# 這支跟 run_experiment.sh 用同一個資料集，差別是這支要在「你自己有 GPU/WSLg 存取權限的終端機」
# 執行才看得到畫面──不要透過自動化工具跑。
#
# 用法:
#   ./view_experiment.sh <config.yaml>
#
# 範例:
#   ./view_experiment.sh configs/exp_我的實驗.yaml

set -euo pipefail
ROOT="/home/david/VSLAM_package"
DATASET="$ROOT/datasets/rgbd_dataset_freiburg1_xyz"
RUN_BIN="$ROOT/stella_vslam_examples/build/run_tum_rgbd_slam"

if [ $# -ne 1 ]; then
    echo "用法: $0 <config.yaml>" >&2
    exit 1
fi

CONFIG="$1"

export LD_LIBRARY_PATH="$ROOT/local_install/lib:${LD_LIBRARY_PATH:-}"

echo "=== 開視窗跑 $CONFIG ==="
echo "(視窗跳出後可以自由旋轉/縮放 3D 地圖；跑完影格不會自動關視窗，看完自己關掉即可)"
"$RUN_BIN" \
    -v "$ROOT/vocab/orb_vocab.fbow" \
    -d "$DATASET" \
    -c "$CONFIG" \
    --no-sleep \
    --viewer pangolin_viewer
