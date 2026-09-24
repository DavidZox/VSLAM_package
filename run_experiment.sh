#!/usr/bin/env bash
# 快速參數實驗工具:給一個 config 檔 + 一個實驗名稱,跑完自動印出跟 ground truth 的 APE 比較，
# 並把兩種「圖」存進 datasets/eval_<實驗名稱>/ 底下，方便之後寫報告用：
#   frames/*.png              每隔 N 幀存一張「當下影格疊 ORB 特徵點」的標註影像（headless，不用開視窗）
#   trajectory_comparison_*.png  估計軌跡 vs ground truth 的疊圖（xy 平面圖 + xyz 分量圖等）
#
# 用法:
#   ./run_experiment.sh <config.yaml> <實驗名稱> [--video]
#
# 範例:
#   cp configs/baseline.yaml configs/exp_fast15.yaml
#   # 編輯 configs/exp_fast15.yaml，把 Feature.ini_fast_threshold 從 20 改成 15
#   ./run_experiment.sh configs/exp_fast15.yaml exp_fast15          # 快速版，只存數字 + 每 30 幀一張圖
#   ./run_experiment.sh configs/exp_fast15.yaml exp_fast15 --video  # 加存一支完整的追蹤過程影片(較慢、檔案較大)
#
# 固定用同一個資料集(TUM RGBD freiburg1_xyz)當測試基準，方便橫向比較不同參數組合。
# 第一次跑會自動存一份 configs/baseline.yaml（未修改過的官方 TUM_RGBD_mono_1.yaml）當對照組。

set -euo pipefail
ROOT="/home/david/VSLAM_package"
DATASET="$ROOT/datasets/rgbd_dataset_freiburg1_xyz"
RUN_BIN="$ROOT/stella_vslam_examples/build/run_tum_rgbd_slam"

if [ $# -lt 2 ] || [ $# -gt 3 ]; then
    echo "用法: $0 <config.yaml> <實驗名稱> [--video]" >&2
    exit 1
fi

CONFIG="$1"
NAME="$2"
WANT_VIDEO="${3:-}"
OUT_DIR="$ROOT/datasets/eval_${NAME}"

export LD_LIBRARY_PATH="$ROOT/local_install/lib:${LD_LIBRARY_PATH:-}"
export PATH="$HOME/.local/bin:$PATH"

if [ ! -f "$ROOT/configs/baseline.yaml" ]; then
    mkdir -p "$ROOT/configs"
    cp "$ROOT/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_1.yaml" "$ROOT/configs/baseline.yaml"
    echo "已建立 configs/baseline.yaml 當作對照組(官方預設值，未修改)"
fi

mkdir -p "$OUT_DIR/frames"

VIDEO_ARGS=()
if [ "$WANT_VIDEO" = "--video" ]; then
    VIDEO_ARGS=(--video-output "$OUT_DIR/tracking.mp4")
fi

echo "=== 執行 $NAME (config: $CONFIG) ==="
"$RUN_BIN" \
    -v "$ROOT/vocab/orb_vocab.fbow" \
    -d "$DATASET" \
    -c "$CONFIG" \
    --no-sleep --auto-term \
    --eval-log-dir "$OUT_DIR" \
    --frame-output-dir "$OUT_DIR/frames" \
    --frame-output-interval 30 \
    "${VIDEO_ARGS[@]}" \
    --viewer none \
    --log-level warn

if [ "$WANT_VIDEO" = "--video" ]; then
    echo "追蹤過程影片: $OUT_DIR/tracking.mp4"
fi

echo
echo "=== $NAME 對照 ground truth 的 APE(公尺) ==="
evo_ape tum "$DATASET/groundtruth.txt" "$OUT_DIR/frame_trajectory.txt" -a

echo
echo "=== 存軌跡疊圖到 $OUT_DIR/trajectory_comparison_*.png ==="
MPLBACKEND=Agg evo_traj tum "$OUT_DIR/frame_trajectory.txt" --ref="$DATASET/groundtruth.txt" \
    -a --plot_mode=xy --save_plot "$OUT_DIR/trajectory_comparison.png" > /dev/null 2>&1 || true
ls "$OUT_DIR"/frames/*.png 2>/dev/null | head -1 > /dev/null \
    && echo "特徵點標註影格: $OUT_DIR/frames/ ($(ls "$OUT_DIR"/frames/*.png | wc -l) 張)"
ls "$OUT_DIR"/trajectory_comparison_*.png 2>/dev/null

if [ -f "$ROOT/datasets/eval_baseline/frame_trajectory.txt" ] && [ "$NAME" != "baseline" ]; then
    echo
    echo "=== 對照組 baseline 的 APE(公尺),方便比較 ==="
    evo_ape tum "$DATASET/groundtruth.txt" "$ROOT/datasets/eval_baseline/frame_trajectory.txt" -a
fi
