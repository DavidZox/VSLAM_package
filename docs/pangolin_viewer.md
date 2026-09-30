# PangolinViewer 安裝(`view_experiment.sh` 開視窗用)

`build.sh` 沒有包含這一段,`pangolin/`、`pangolin_viewer/` 也不進版控,所以新 clone 的電腦要手動裝一次。
只跑 `run_experiment.sh`(不開視窗)或 ROS2 部署(`--viewer none`)的話不需要。

版本固定在這台開發機驗證過的 commit:Pangolin `3667bb76`(2026-08-19)、pangolin_viewer `1e023353`(2024-01-09)。

在 `VSLAM_package` 根目錄執行(要先跑過 `./build.sh`):

```bash
source env.sh
sudo apt install -y libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \
  libegl1-mesa-dev libc++-dev libepoxy-dev libglew-dev

# 資料夾名稱用小寫 pangolin,才會被 .gitignore 排除
git clone https://github.com/stevenlovegrove/Pangolin.git pangolin
git -C pangolin checkout 3667bb76edcc3e4111d2bee7935ce204f04026d9
cmake -S pangolin -B pangolin/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$VSLAM_ROOT/local_install" -DCMAKE_PREFIX_PATH="$VSLAM_ROOT/local_install" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF
cmake --build pangolin/build -j"$(nproc)" && cmake --install pangolin/build

git clone https://github.com/stella-cv/pangolin_viewer.git pangolin_viewer
git -C pangolin_viewer checkout 1e0233534055ea3c184dbf6f8b254a0c6c25cee4
git -C pangolin_viewer submodule update --init --recursive
cmake -S pangolin_viewer -B pangolin_viewer/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$VSLAM_ROOT/local_install" -DCMAKE_PREFIX_PATH="$VSLAM_ROOT/local_install" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build pangolin_viewer/build -j"$(nproc)" && cmake --install pangolin_viewer/build

# 讓已經 build 好的 stella_vslam_examples 重新偵測、連結 pangolin_viewer
cmake -S stella_vslam_examples -B stella_vslam_examples/build -DCMAKE_PREFIX_PATH="$VSLAM_ROOT/local_install"
cmake --build stella_vslam_examples/build -j"$(nproc)"
```

裝好之後 `./view_experiment.sh configs/<某個 config>.yaml` 就會開視窗,同時顯示「當下影格疊 ORB 特徵點」跟
「3D 地圖/相機軌跡即時建構過程」。

⚠️ 視窗要在有 GPU/WSLg 存取權限的終端機才看得到畫面。遇到 `MESA`/`ZINK`/`dri2` 相關錯誤是 WSLg GPU
驅動設定問題,可以考慮改裝不需要 OpenGL 的 SocketViewer(用瀏覽器看)。
