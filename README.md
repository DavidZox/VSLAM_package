# VSLAM_package
自訂義開發的VSLAM專案

核心演算法用 [stella_vslam](https://github.com/stella-cv/stella_vslam)(BSD 2-Clause,可商用),ROS2 部署層用
[stella_vslam_ros](https://github.com/stella-cv/stella_vslam_ros)。這兩個 + 它們的依賴 `g2o`、
`stella_vslam_examples`(實際的 `run_video_slam` 等測試執行檔在這個獨立 repo,不在 `stella_vslam` 本體裡)都是以
**git submodule** 方式引用,只記錄版本指標,不把對方的原始碼內容存進這個 repo。

## 首次取得

```bash
git clone --recurse-submodules <this-repo-url>
# 如果已經 clone 過忘了帶 --recurse-submodules：
git submodule update --init --recursive
```

## 本地端核心算法建置(不含 ROS2)

一次性系統依賴(需要 sudo，只需跑一次)：

```bash
sudo apt update
sudo apt install -y build-essential pkg-config cmake git wget curl unzip \
  libopencv-dev libeigen3-dev libyaml-cpp-dev libsuitesparse-dev libsqlite3-dev
```

之後都不需要 sudo：

```bash
./build.sh          # 編譯 g2o → stella_vslam → stella_vslam_examples 到 local_install/，並下載 ORB 詞彙檔
source env.sh        # 設定 LD_LIBRARY_PATH 等執行期環境變數
"$RUN_VIDEO_SLAM" -v "$VOCAB_FILE" -m /path/to/video.mp4 -c /path/to/camera.yaml --viewer none
```

`local_install/`、`vocab/`、各 submodule 底下的 `build/` 都不進版控（見 `.gitignore`），重新 `./build.sh` 即可重建，
不需要、也不應該把這些編譯產物或第三方原始碼 commit 上來。

## ROS2 部署（`stella_vslam_ros`）

尚未開始建置，先整理過介接資訊：訂閱 `camera/image_raw`（單目）/ `camera/left|right/image_raw`（雙目）/
`camera/color|depth/image_raw`（RGBD），發布 `~/camera_pose`；`ament_cmake` 標準套件，執行檔名叫 `system`。
待本地端核心算法測試驗證完再往這邊推進。
