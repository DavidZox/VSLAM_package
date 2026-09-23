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
- [ ] **還沒拿真實影片跑過完整的 SLAM 流程**(目前只驗證了「程式能跑」,還沒驗證追蹤/建圖品質)
- [ ] 還沒裝任何 3D 視覺化介面(Pangolin/SocketViewer 等),目前是純文字輸出的 headless 模式
- [ ] `stella_vslam_ros`(ROS2 部署層)只做了介接資訊調查,還沒開始建置——待本地端核心算法驗證過再推進

## 資料夾結構

```
VSLAM_package/
├── README.md            本文件
├── build.sh              一鍵建置腳本(g2o → stella_vslam → stella_vslam_examples）
├── env.sh                執行前 source,設定 LD_LIBRARY_PATH 等環境變數
├── .gitmodules            四個上游 repo 的 submodule 註冊資訊
├── .gitignore
│
├── g2o/                  【submodule】RainerKuemmerle/g2o,stella_vslam 的圖優化後端依賴
├── stella_vslam/          【submodule】核心 VSLAM 演算法庫
├── stella_vslam_examples/ 【submodule】本地端測試執行檔原始碼(run_video_slam 等,見下方說明)
├── stella_vslam_ros/       【submodule】ROS2 部署層,尚未建置
│
├── local_install/        （不進版控）g2o + stella_vslam 的 from-source 安裝結果,by build.sh 產生
└── vocab/                （不進版控）ORB 詞彙檔 orb_vocab.fbow,by build.sh 下載
```

`local_install/`、`vocab/`、各 submodule 底下自己的 `build/` 都是建置產物或第三方內容,不進版控,重新
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

### 之後想加視覺化介面

目前 `--viewer` 只能傳 `none`。要加即時 3D 畫面,兩個選項(擇一,裝好後重新 `./build.sh` 會自動偵測到並連結):

- **PangolinViewer**——傳統桌面 OpenGL 視窗;WSL2 環境需要 WSLg 圖形轉發才看得到畫面。
- **SocketViewer**——用瀏覽器看(Node.js 架一個本地網頁伺服器),比較不挑執行環境。

裝法見 [stella_vslam 官方文件](https://stella-cv.readthedocs.io/en/latest/installation.html) 的
Viewer 章節,兩者都不需要動 `build.sh` 的核心邏輯。

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
