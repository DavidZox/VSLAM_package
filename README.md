# VSLAM_package

自訂義開發的 VSLAM 專案。核心演算法用 [stella_vslam](https://github.com/stella-cv/stella_vslam)
(BSD 2-Clause,確認可商用,2021 年從 OpenVSLAM fork 出來就是為了徹底拿掉跟 ORB-SLAM2 的 GPL 相似性),
支援單目/雙目/RGBD,相機模型可換(針孔/魚眼/全景)。未來部署層用它的 ROS2 套件
[stella_vslam_ros](https://github.com/stella-cv/stella_vslam_ros)。

## 目前狀態

- [x] 調查 `stella_vslam` / `stella_vslam_ros` 授權與架構,確認可商用
- [x] 本地端核心算法建置環境(`g2o` → `stella_vslam` → `stella_vslam_examples`)已建好、`run_video_slam`
      驗證能正常啟動並正確連結所有函式庫
- [x] 四個上游依賴改用 **git submodule** 釘版本管理(見下方「版控說明」),`build.sh` 記錄完整建置步驟
- [x] **拿公開資料集(TUM RGBD)驗證過完整 SLAM 流程,包含追蹤精度**(見下方「測試紀錄」),不只是「程式能跑」
- [ ] 還沒拿自己的相機/影片實測(目前只用公開資料集驗證過)
- [x] **PangolinViewer 裝好了**(見「視覺化介面」章節),可以即時看特徵點 + 3D 地圖建構過程——但只在
      建置那台機器驗證過編譯/運算正常,視窗畫面實際算繪需要在有 GPU 存取權限的終端機自行確認
- [x] **整條 pipeline ~105 個可調參數整理好了**(見 `docs/tunable_parameters.md`),含官方範例本身的
      bug 跟寫死常數清單,可以系統性地一個一個試
- [x] **`run_experiment.sh`/`view_experiment.sh` 自動化實驗工具做好了**(見「自動化參數實驗」章節),
      改一個參數 → 跑一次 → 自動算 APE、存特徵點標註影格/追蹤影片/軌跡疊圖到 `datasets/eval_<實驗名稱>/`,
      目前已經用 `configs/` 底下 5 組參數實驗(scale_factor、num_levels、min_size、baseline_dist_thr_ratio、
      故意調錯的 intrinsics)橫向比較過
- [ ] `stella_vslam_ros`(ROS2 部署層)只做了介接資訊調查,還沒開始建置——待本地端核心算法驗證過再推進
- [ ] ZED 雙目相機規格只先做了設定範本跟接入方式調查(見 `docs/zed_stereo.md`),還沒有實體相機可以測

## 資料夾結構

```
VSLAM_package/
├── README.md            本文件
├── build.sh              一鍵建置腳本(g2o → stella_vslam → stella_vslam_examples）
├── env.sh                執行前 source,設定 LD_LIBRARY_PATH 等環境變數
├── .gitmodules            四個上游 repo 的 submodule 註冊資訊
├── .gitignore
├── run_experiment.sh      自動化參數實驗:跑一組 config、算 APE、存標註影格/影片/軌跡圖(見下方說明)
├── view_experiment.sh     跟 run_experiment.sh 用同一組資料,改成開 PangolinViewer 視窗即時看
├── configs/               自己專案的相機設定檔(跟 stella_vslam/example/ 的官方範例分開放)
│   ├── baseline.yaml          對照組,官方 TUM_RGBD_mono_1.yaml 原封不動的副本
│   ├── exp_scale_factor.yaml  實驗:scale_factor 1.2 → 1.5
│   ├── exp_num_levels.yaml    實驗:num_levels 8 → 4
│   ├── exp_min_size.yaml      實驗:min_size 800 → 300
│   ├── exp_baseline_ratio.yaml 實驗:baseline_dist_thr_ratio 0.02 → 0.1
│   ├── exp_bad_intrinsics.yaml 實驗:故意調錯 fx/fy(體會校正錯誤的影響,不是找最佳值)
│   └── ZED_stereo.yaml    ZED 雙目相機設定範本,細節見 docs/zed_stereo.md
├── docs/
│   ├── zed_stereo.md          ZED 雙目相機校正參數 + 接入方式筆記
│   └── tunable_parameters.md  整條 pipeline ~105 個可調參數對照表
│
├── g2o/                  【submodule】RainerKuemmerle/g2o,stella_vslam 的圖優化後端依賴
├── stella_vslam/          【submodule】核心 VSLAM 演算法庫
├── stella_vslam_examples/ 【submodule】本地端測試執行檔原始碼(run_video_slam 等,見下方說明)
├── stella_vslam_ros/       【submodule】ROS2 部署層,尚未建置
│
├── local_install/        （不進版控）g2o + stella_vslam 的 from-source 安裝結果,by build.sh 產生
├── vocab/                （不進版控）ORB 詞彙檔 orb_vocab.fbow,by build.sh 下載
└── datasets/             （不進版控）測試用資料集 + run_experiment.sh 的輸出,見下方說明
    └── eval_<實驗名稱>/   每組實驗一個資料夾(run_experiment.sh 自動建立)
        ├── frame_trajectory.txt / keyframe_trajectory.txt / track_times.txt   估計軌跡與耗時(TUM 格式)
        ├── frames/*.png                每隔 N 幀存一張「當下影格疊 ORB 特徵點」的標註影像
        ├── tracking.mp4                 加 --video 才會有,完整追蹤過程影片
        └── trajectory_comparison_*.png  估計軌跡 vs ground truth 疊圖(evo_traj 產生)
```

`local_install/`、`vocab/`、`datasets/`、各 submodule 底下自己的 `build/` 都是建置產物、下載資產或第三方內容,不進版控,重新
跑 `build.sh` 就能重建(細節見文末「版控說明」)。

## 使用方式:本地端核心算法測試

### 1. 取得原始碼

```bash
git clone --recurse-submodules https://github.com/DavidZox/VSLAM_package.git
cd VSLAM_package
# 如果是已經 clone 過的舊資料夾、忘了帶 --recurse-submodules：
git submodule update --init --recursive
```

### 2. 安裝系統依賴(一次性,需要 sudo)

```bash
sudo apt update
sudo apt install -y build-essential pkg-config cmake git wget curl unzip \
  libopencv-dev libeigen3-dev libyaml-cpp-dev libsuitesparse-dev libsqlite3-dev
```

`g2o` 沒有對應的可用 apt 套件(Ubuntu 的 `libg2o-dev` 沒有附 CMake config 檔,`stella_vslam` 的建置腳本
找不到),所以固定從原始碼編譯;OpenCV / Eigen / yaml-cpp / SuiteSparse / SQLite3 則直接吃系統套件,
比照官方文件從原始碼編譯快很多。

### 3. 建置(之後都不需要 sudo)

```bash
./build.sh
```

會依序編譯安裝 `g2o` → `stella_vslam` → `stella_vslam_examples` 到 `local_install/`(local prefix,不動系統路徑),
並下載 ORB 詞彙檔到 `vocab/orb_vocab.fbow`。目前**沒有編譯任何 3D 檢視器**(Pangolin/SocketViewer 等),
先求最快建置成功,純文字輸出;之後要看即時軌跡/點雲畫面再另外加裝(見下方)。

### 4. 準備測試素材

需要兩樣東西:

1. **一段影片**(手機錄的即可,mp4/avi 皆可讀)——SLAM 需要「有平移的相機運動」才能算出深度,原地
   轉圈或純旋轉沒有視差、初始化不了,建議走動錄個 10~20 秒。
2. **一份相機參數 YAML**——最少要有焦距跟主點(單位:像素)。`stella_vslam/example/` 底下有各資料集的
   範例可以參考(例如 `stella_vslam/example/kitti/KITTI_mono_03.yaml`),關鍵欄位:

   ```yaml
   Camera:
     setup: "monocular"      # 這階段先用單目
     model: "perspective"    # 一般手機/webcam 用這個,魚眼鏡頭用 "fisheye"
     fx: <焦距 x,像素>
     fy: <焦距 y,像素>
     cx: <主點 x,通常約影像寬度一半>
     cy: <主點 y,通常約影像高度一半>
     k1: 0.0                 # 有畸變校正參數的話填,沒有就先留 0
     k2: 0.0
     p1: 0.0
     p2: 0.0
     k3: 0.0
     fps: <影片幀率>
     cols: <影像寬>
     rows: <影像高>
     color_order: "RGB"
   ```

   不知道 fx/fy/cx/cy 的話,先抓「影像寬度」當粗略 fx/fy 估計值、cx/cy 用寬高各半代入——不精準但足夠
   先驗證流程跑不跑得通;正式要準的話之後可以用 OpenCV 棋盤格校正。

### 5. 執行

```bash
source env.sh
"$RUN_VIDEO_SLAM" \
  -v "$VOCAB_FILE" \
  -m /path/to/你的影片.mp4 \
  -c /path/to/你的camera.yaml \
  --viewer none
```

終端機會印出追蹤/建圖統計。其他常用參數:

| 參數 | 用途 |
|---|---|
| `--map-db-out output.db` | 執行完存下地圖資料庫,之後可用 `-i` 載入做 relocalization |
| `--eval-log-dir <目錄>` | 存軌跡跟每幀耗時,用來評估追蹤品質/效能 |
| `--frame-skip N` | 跳幀處理,影片幀率太高或想加速測試時用 |
| `--without-vocab` | 不載入詞彙檔(跳過 loop closure/relocalization,純測 tracking) |
| `--no-sleep` | 不依照影片實際 fps 等待,盡快跑完 |

`stella_vslam_examples/build/` 底下還有 `run_image_slam`(單張影像序列)、`run_camera_slam`(即時攝影機)、
`run_kitti_slam`/`run_euroc_slam`/`run_tum_rgbd_slam`(標準資料集,用法可以參考 `stella_vslam/example/`
底下對應的 YAML)。

### 視覺化介面:PangolinViewer

已經裝好([stella_vslam 官方文件](https://stella-cv.readthedocs.io/en/latest/installation.html) 的
Viewer 章節那個版本),`--viewer` 可以傳 `pangolin_viewer` 了,會開一個視窗同時顯示「當下影格疊 ORB
特徵點」跟「3D 地圖/相機軌跡即時建構過程」:

```bash
source env.sh
"${VSLAM_ROOT}/stella_vslam_examples/build/run_tum_rgbd_slam" \
  -v "$VOCAB_FILE" \
  -d datasets/rgbd_dataset_freiburg1_xyz \
  -c stella_vslam/example/tum_rgbd/TUM_RGBD_mono_1.yaml \
  --no-sleep --viewer pangolin_viewer
```

安裝方式(供其他機器重建參考,`build.sh` 目前還沒自動包含這段,是額外手動裝的):

```bash
sudo apt install -y libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \
  libegl1-mesa-dev libc++-dev libepoxy-dev libglew-dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cmake -S Pangolin -B Pangolin/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$VSLAM_ROOT/local_install" -DCMAKE_PREFIX_PATH="$VSLAM_ROOT/local_install" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF
cmake --build Pangolin/build -j"$(nproc)" && cmake --install Pangolin/build

git clone --recursive https://github.com/stella-cv/pangolin_viewer.git
cmake -S pangolin_viewer -B pangolin_viewer/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$VSLAM_ROOT/local_install" -DCMAKE_PREFIX_PATH="$VSLAM_ROOT/local_install" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build pangolin_viewer/build -j"$(nproc)" && cmake --install pangolin_viewer/build

# 讓已經 build 好的 stella_vslam_examples 重新偵測、連結 pangolin_viewer：
cmake -S stella_vslam_examples -B stella_vslam_examples/build -DCMAKE_PREFIX_PATH="$VSLAM_ROOT/local_install"
cmake --build stella_vslam_examples/build -j"$(nproc)"
```

⚠️ 這幾支 build 指令我這邊(工具執行環境)沒有 GPU 裝置(`/dev/dri` 不存在),雖然編譯跟 SLAM 運算都
正常跑完,但沒辦法確認視窗畫面實際算繪是否正常——**這個指令要在你自己有 WSLg/GPU 存取權限的終端機
執行才能真的看到畫面**,不要透過自動化工具跑。如果你那邊也遇到 `MESA`/`ZINK`/`dri2` 相關錯誤,是
WSLg GPU 驅動設定問題,可以考慮改裝不需要 OpenGL 的 SocketViewer(瀏覽器看)當替代方案。

### 系統性調參數:`docs/tunable_parameters.md`

整條 pipeline(前端特徵擷取、初始化、追蹤、建圖/local BA、迴環偵測、global BA/pose graph、System)
所有能調的 YAML 參數,包含官方範例都沒列出來、要查原始碼才知道存在的隱藏參數,總共約 105 個,整理在
[`docs/tunable_parameters.md`](docs/tunable_parameters.md)。裡面還記錄了一個 stella_vslam 官方範例本身
的 bug(`num_grid_cols`/`num_grid_rows` 設在錯的 YAML 區塊,一直沒生效過)跟一份完全寫死、無法透過設定
檔調整的常數清單。文件最後也有「怎麼有系統地做參數實驗」的建議流程(改一個參數 → 用 `evo_ape` 量化
比較 → 換下一個),搭配上面「測試紀錄」章節已經裝好的 `evo`。

### 自動化參數實驗:`run_experiment.sh` / `view_experiment.sh`

手動一步步跑 `run_tum_rgbd_slam` + `evo_ape` + `evo_traj` 太瑣碎,`run_experiment.sh` 把整套流程包成
一個指令,固定用 `datasets/rgbd_dataset_freiburg1_xyz` 當測試基準,方便不同參數組合橫向比較:

```bash
# 1. 複製一份 config,改想測的參數
cp configs/baseline.yaml configs/exp_我的實驗.yaml

# 2. 跑實驗(數字 + 每 30 幀一張標註圖)
./run_experiment.sh configs/exp_我的實驗.yaml exp_我的實驗

# 3. 想要完整影片再加 --video(較慢、檔案較大,適合最後要拿去寫報告的那幾組再加)
./run_experiment.sh configs/exp_我的實驗.yaml exp_我的實驗 --video
```

跑完會自動:
1. 印出這組跟 ground truth 的 APE(SE(3) Umeyama 對齊,max/mean/median/min/rmse/sse)
2. 印出 `configs/baseline.yaml`(對照組)的 APE 供比較(baseline 自己不會重複比較)
3. 把追蹤過程的標註影格(`frames/*.png`)、完整影片(`tracking.mp4`,加 `--video` 才有)、軌跡疊圖
   (`trajectory_comparison_*.png`)都存到 `datasets/eval_<實驗名稱>/` 底下,可以直接拿去寫報告

想「實際看到畫面」(而不只是看數字跟事後存的靜態圖),用 `view_experiment.sh` 開 PangolinViewer 視窗即時看
特徵點追蹤 + 3D 地圖建構過程——這支跟前面「視覺化介面」章節一樣,**要在你自己有 GPU/WSLg 存取權限的終端機
執行**,不要透過自動化工具跑:

```bash
./view_experiment.sh configs/exp_我的實驗.yaml
```

目前 `configs/` 底下已經準備好 5 組單一變數的實驗(每份 config 開頭的註解都寫了改了什麼、預期會怎樣),
對應 `docs/tunable_parameters.md` 裡挑出來的幾個代表性參數:`exp_scale_factor`(金字塔縮放倍率)、
`exp_num_levels`(金字塔層數)、`exp_min_size`(特徵點分佈網格大小)、`exp_baseline_ratio`(三角化新地標
要求的最小相機移動量)、`exp_bad_intrinsics`(故意調錯相機焦距,體會校正誤差的影響)。

⚠️ **一個重要前提,解讀數字前一定要記得**:stella_vslam 初始化用的 RANSAC 預設沒有固定種子,所以就算
**完全不改參數、同一份 config 重跑兩次**,APE 數字也會有小幅波動;實際測下來,效果不夠大、方向不夠穩定
的參數(例如 `min_size`、`baseline_dist_thr_ratio` 這種調整幅度較保守的實驗)確實出現過「這次變好、下次
變差」的情況,幅度大到足以讓結論整個翻轉。**只跑一次就下「這個參數比較好」的結論並不可靠**——效果夠大、
方向跨多次重跑都穩定的(例如 `num_levels` 調小、`bad_intrinsics` 這兩組目前每次重跑都是明顯變差)比較
能信;效果不上不下的,建議同一組 config 多跑個 2~3 次看數值範圍,不要只憑單次結果比較,或考慮把
`Initializer.use_fixed_seed` 設成 `true` 固定亂數種子讓結果可重現(細節見 `docs/tunable_parameters.md`)。

## 測試紀錄

### TUM RGBD `freiburg1_xyz`(單目,含 ground truth 精度驗證)

EuRoC 官方資料集主機(`robotics.ethz.ch`)目前連不上,改用 [TUM RGBD](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)
的 `freiburg1_xyz` 序列(手持相機沿 X/Y/Z 軸平移,796 幀,自帶 motion-capture ground truth)驗證整條
pipeline。下面是最早手動跑的第一次驗證紀錄;之後系統性比較不同參數改用上面「自動化參數實驗」章節的
`run_experiment.sh`,不用再照這樣一步步手動下指令:

```bash
source env.sh
"${VSLAM_ROOT}/stella_vslam_examples/build/run_tum_rgbd_slam" \
  -v "$VOCAB_FILE" \
  -d /path/to/rgbd_dataset_freiburg1_xyz \
  -c "${VSLAM_ROOT}/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_1.yaml" \
  --no-sleep --auto-term \
  --eval-log-dir /path/to/output目錄 \
  --viewer none
```

**結果**:單目初始化成功、796 幀全部處理完、產生 19 個關鍵幀;平均每幀追蹤耗時 ~10ms(遠低於這份
資料集 30fps 的 33ms 預算,代表這台機器的算力對即時處理綽綽有餘)。拿
[`evo`](https://github.com/MichaelGrupp/evo)(`pip install evo`)對照官方 ground truth 算絕對軌跡誤差
(APE,SE(3) Umeyama 對齊,單位公尺):

| max | mean | median | min | rmse | std |
|---|---|---|---|---|---|
| 0.127 | 0.053 | 0.049 | 0.001 | 0.060 | 0.028 |

RMSE 約 6 公分,對一段沒有特別調參的單目 quick-run 來說是合理的精度水準。軌跡疊圖(藍色是估計軌跡,
灰色虛線是 ground truth,形狀吻合 `freiburg1_xyz` 這個序列刻意設計的十字型 X/Y/Z 平移運動):

```bash
evo_ape tum groundtruth.txt frame_trajectory.txt -a
evo_traj tum frame_trajectory.txt --ref=groundtruth.txt -a --plot_mode=xy --save_plot compare.png
```

### 輸出檔案怎麼看

`--eval-log-dir` 指定的目錄底下會有:

| 檔案 | 內容 |
|---|---|
| `frame_trajectory.txt` | 每一幀的估計位姿,TUM 格式:`timestamp tx ty tz qx qy qz qw`(位置 + 四元數姿態) |
| `keyframe_trajectory.txt` | 同格式,只列關鍵幀(這次是 19 筆) |
| `track_times.txt` | 每一幀實際花多少時間追蹤,拿來評估效能 |

`groundtruth.txt`(資料集自帶)也是同樣的 `timestamp tx ty tz qx qy qz qw` 格式,所以能直接拿
`evo_ape`/`evo_rpe` 比對,不用自己轉檔。

執行過程中出現過幾行 `Cholesky failed` 警告(來自 g2o 做 bundle adjustment 時),只發生在最開始
建圖的瞬間(初始地圖只有 66 個點,約束不足導致 Hessian 矩陣暫時病態),後續建圖/追蹤沒有再出現、
結果也正常,先記錄下來,之後如果在其他資料集又看到同樣警告且影響到結果品質,再回頭深究。

## 已知眉角(踩過的雷)

- **`run_video_slam` 等執行檔實際上不在 `stella_vslam` 本體裡**,官方 `stella_vslam` README 寫的路徑
  (`./example/*.cc`)已經過時——現在搬到獨立的 `stella_vslam_examples` repo,`stella_vslam/example/`
  只剩各資料集的 YAML 設定檔。
- **這台機器的 CMake(4.x)會直接拒絕** `stella_vslam`/`stella_vslam_examples` 宣告的
  `cmake_minimum_required(VERSION 3.1)`,要加 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`(已寫進 `build.sh`)。
- **g2o 裝出來的 `g2oConfig.cmake` 有問題**:不管有沒有開 `G2O_USE_OPENGL`,都無條件
  `find_dependency(OpenGL)`,導致 `stella_vslam` 的 `find_package(g2o)` 失敗。`stella_vslam` 實際用到的
  g2o 元件都不需要 OpenGL,`build.sh` 裝完 g2o 後會自動把這行刪掉。
- **Ubuntu 的 `libg2o-dev` apt 套件不能用**——下載下來檢查過,裡面沒附 CMake config 檔(`g2oConfig.cmake`
  等),`stella_vslam` 的 `find_package(g2o REQUIRED ...)` 找不到對應的 imported target,只能從原始碼編譯。
- **`stella_vslam_examples` 自己也有 submodule**(`3rd/filesystem`),`git submodule update --init
  --recursive` 要用 `--recursive`,不然編譯會在 `#include <ghc/filesystem.hpp>` 那行直接失敗。

## 未來部署方式:ROS2(`stella_vslam_ros`)

還沒開始建置,先記錄調查過的介接資訊,供之後串接 `ros2-web-app` 工作流程參考:

- **套件型態**:標準 `ament_cmake` 套件,跟現有 ROS2 workspace 一致,`colcon build` 即可,依賴
  `stella_vslam`(要能被 CMake/ament 找到)+ `rclcpp`、`rclcpp_components`、`cv_bridge`、`image_transport`、
  `tf2` 全家桶(`tf2_eigen`/`tf2_geometry_msgs`/`tf2_msgs`/`tf2_ros`)、`geometry_msgs`、`nav_msgs`、
  `sensor_msgs`、`message_filters`、`rosbag2_cpp`、`libgoogle-glog-dev`。
- **訂閱 topic**(依相機設定擇一):
  - 單目:`camera/image_raw`
  - 雙目:`camera/left/image_raw`、`camera/right/image_raw`
  - RGBD:`camera/color/image_raw`、`camera/depth/image_raw`
- **發布**:`~/camera_pose`;可選發布 TF(`publish_tf` 參數),搭配 `odom_frame`/`map_frame`/
  `robot_base_frame`/`camera_frame`/`transform_tolerance` 等參數調整座標系。
- **執行檔**:註冊為 composable node,`ros2 run stella_vslam_ros system`。另外有一個
  `run_slam_offline` 可以直接讀 rosbag2 檔案跑,不用即時訂閱 topic——很適合拿現場錄好的 rosbag 離線
  驗證,不用真的接上相機 topic。
- **待辦**(等本地端核心算法測試確認品質沒問題後再推進):建置 `stella_vslam_ros`、決定相機來源
  (實體相機 topic 或 rosbag 離線)、把 `stella_vslam` 的 `local_install/` 讓 ROS2 build 系統找得到、
  規劃跟 `ros2-web-app` 既有節點(如 `fih_rmf_system`)的整合方式。
- **相機規劃**:未來預計用 ZED 雙目相機——校正參數怎麼拿、`configs/ZED_stereo.yaml` 範本怎麼填、
  以及目前發現的一個技術限制(本地端 `run_camera_slam` 的 stereo 模式接不了 ZED 這種單一 USB 裝置,
  建議直接走 ZED 官方 `zed-ros2-wrapper` 接 `stella_vslam_ros`),完整記錄在 `docs/zed_stereo.md`。

## 版控說明

`g2o`/`stella_vslam`/`stella_vslam_examples`/`stella_vslam_ros` 都是**別人的 open source repo**,用
git submodule 引用(`.gitmodules` 記錄版本指標,只占幾 KB),而不是整包內容存進這個 repo——早先曾經
不小心把建置產物、下載的詞彙檔、submodule 內容整包 commit 進去,單一 push 膨脹到 100MB+,在較慢的
連線環境下會 push 逾時失敗;改成 submodule 後 `.git` 大小回到幾百 KB。`build.sh` 是重建
`local_install/`/`vocab/` 的唯一依據,這兩個目錄永遠不該手動加入版控。

四個 submodule 的 `.gitmodules` URL 指向的是**自己帳號底下 fork 的版本**(`DavidZox/g2o`、
`DavidZox/stella_vslam` 等),不是直接指向 `stella-cv`/`RainerKuemmerle` 原始 repo——避免上游哪天改版本、
砍分支甚至整個 repo 消失,導致這裡的 submodule 指標抓不到對應 commit。每個 submodule 資料夾裡另外設了
`upstream` remote 指回原始 repo。

### 修改 submodule 裡面的內容時,一定要「兩層都 push」

`stella_vslam`/`g2o`/`stella_vslam_examples`/`stella_vslam_ros` 每個資料夾裡都是獨立的 git repo,
`VSLAM_package` 只在自己的歷史裡記一個「這個資料夾現在對應到 submodule 的哪個 commit」的指標,不會記錄
裡面實際改了哪些檔案。所以只要改動到 submodule 裡面的檔案(例如以後改 `stella_vslam_ros` 接自己的
ROS2 系統、或是同步上游更新),流程一定分兩層,**順序不能顛倒**:

```bash
# 第 1 層：在 submodule 自己的資料夾裡先 commit + push
cd stella_vslam_ros
git add -A && git commit -m "..."
git push origin main          # 推到自己的 fork,例如 DavidZox/stella_vslam_ros

# 第 2 層：確定第 1 層真的推上去之後，回到 VSLAM_package 根目錄
cd ..
git add stella_vslam_ros      # 只是把指標更新到剛剛那個新 commit，不是加檔案內容
git commit -m "Bump stella_vslam_ros submodule pin"
git push origin main
```

**為什麼順序不能反過來**:主 repo 記的指標只是一串 commit hash,git 不會檢查這串 hash 是否真的能在
遠端抓到才讓你 commit/push。如果先推了 `VSLAM_package`,指標會指向一個還只存在本機、沒推上 fork 的
commit,等於開了一張空頭支票——之後任何人(包含你自己在別台機器)`git clone --recurse-submodules`
時,git 會照指標去 fork 抓那個 commit,結果抓不到,直接失敗。所以永遠是「先確定 submodule 那層真的
推上去了,再回頭更新主 repo 的指標」。

這套流程**只有在真的動到 submodule 裡面的檔案時才需要**;平常只改 `VSLAM_package` 自己的檔案
(`README.md`、`build.sh`、`env.sh`)是單純一般的 commit + push,不會牽扯到這兩層。

同步上游更新是這套流程的一種情況,差別只在 submodule 那層的來源改成 `upstream`:

```bash
cd stella_vslam_ros
git fetch upstream
git merge upstream/main   # 或 upstream 的預設分支名稱(例如 stella_vslam_ros 是 upstream/ros2)
git push origin main      # 推回自己的 fork
cd ..
git add stella_vslam_ros && git commit -m "Sync stella_vslam_ros with upstream" && git push origin main
```
