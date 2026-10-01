# vslam_bringup

雙目(ZED)VSLAM 部署用的 ROS2 **bringup 套件**——只包 launch 檔、參數 YAML 跟幾支對接/驗證小工具,**不含 SLAM 演算法本身**。
實際運算由 [`stella_vslam_ros`](../stella_vslam_ros/)(submodule)提供,這個套件單純負責「組裝參數、
把相機 topic 接起來、設定 TF」,跟 `ros2-web-app` 那類專案裡常見的 `*_bringup` 套件角色一樣。

✅ **已經在真正的 ROS2 Jazzy 環境驗證過,而且是真的啟動節點,不是只看 `--show-args`**:`colcon_build.sh`
在一個實際跑著的 ROS2 Jazzy 容器裡從乾淨狀態跑過,四個套件全部編譯成功,`ros2 launch vslam_bringup
stereo_vslam.launch.py`(拿掉 `--show-args`,真的執行)也確認 `run_slam` 節點會正常啟動、載入設定跟
詞彙檔、啟動 SLAM 各模組,持續執行不會當掉(還沒接相機,只驗證到「啟動成功、等待影像」)。過程中連帶
修掉 `stella_vslam_ros` 四個真實 bug——ament 環境 hook 重複註冊、`cv_bridge.h`→`.hpp` 改名,以及一個
**只有真的啟動節點才會踩到**的:自訂的 `-r`/`--rectify` 參數跟 ROS2 自己的 `-r`/`--remap` 撞名,
雙目模式的 topic remap 會誤觸發 rectify、讀取一個故意沒有的 YAML 區塊直接把節點弄死。細節見主
`README.md`「已知眉角」。

還沒驗證的只剩「接上真的 ZED 相機」這塊——編譯跟 launch 檔本身沒問題,但實際影像 topic 名稱、TF 設定
都還是紙上規劃,要等實體相機/`zed-ros2-wrapper` 到位才能確認,具體是下面「實際接 ZED 相機運作前」那 3 個地方。

## Isaac Sim 機器人對接(2026-09-30 在模擬裡驗證過)

`fih_humanoid_amr` 的 Isaac Sim 場景(`Scene/`)裡,機器人頭上有一顆 ZED X,Isaac 直接發布左右影像跟
camera_info。這部分**已經實際接起來跑過整圈**:內參、基線、相機 TF、topic 都是從模擬裡量的,不是佔位符。
觀察到的數值、踩到的問題、每次實驗的結果都記錄在 [`../docs/isaac_sim_integration.md`](../docs/isaac_sim_integration.md)。

### 啟動順序

```bash
# 1. Isaac Sim(主機):開場景,等 log 出現 "SimControlNode ready"
~/Gitlab_workspace/fih_humanoid_amr/Scene/run_sim.sh

# 2. 機器人端(amr-base-dev 容器):平常的 start_sim.sh 流程,會先對 /sim_control 送 stop/play 讓模擬開始跑,
#    再 launch start_sim_and_spawn_IsaacSim.launch.py(TF、odom、AMCL、Nav2 都在這裡)

# 3. VSLAM(這個 repo 的根目錄,主機上執行;沒有 ROS2 的主機用容器跑)
./ros2_container.sh up        # 第一次:用 amr_base 映像建 vslam-dev 容器,ROS_DOMAIN_ID 讀 amr_base/domain_id.conf
./ros2_container.sh build     # 第一次或改了程式:容器裡跑 colcon_build.sh
./ros2_container.sh launch    # = ros2 launch vslam_bringup isaac_stereo_vslam.launch.py,可以接 launch 參數
./ros2_container.sh stop      # 結束(送 SIGINT,有給 map_db_out / eval_log_dir 的話這時才存檔)
```

常用的 launch 參數:

| 參數 | 用途 |
|---|---|
| `map_db_out:=<檔案>` | 結束時存地圖(msgpack) |
| `map_db_in:=<檔案> disable_mapping:=true` | 載入地圖只做定位 |
| `eval_log_dir:=<目錄>` | 結束時存 frame/keyframe 軌跡(TUM)跟每幀追蹤耗時 |
| `rviz:=true` | 另外開 RViz(`rviz/isaac_vslam.rviz`):真值 `/odom` vs VSLAM 軌跡、關鍵幀、地圖點、這一幀特徵點的 3D 位置、迴環邊、追蹤影像;2D Pose Estimate 送到 `/vslam/initialpose` |
| `map_parent_frame:=map` | `vslam_map` 改掛在 AMCL 的 `map` 底下(預設掛 `odom`),VSLAM 地圖直接對齊機器人既有地圖 |
| `publish_tf:=true map_frame:=map` | 讓 VSLAM 接手 `map -> odom`(機器人端這時不能再跑 AMCL / slam_toolbox,否則兩邊搶同一段 TF;這條還沒實測) |
| `mask:=none` | 不用本體遮罩(只在換了機器人外觀/手臂姿勢、要重拍遮罩前比較用) |

### 這組設定接了什麼

| 項目 | 檔案 | 內容 |
|---|---|---|
| 影像 | `launch/isaac_stereo_vslam.launch.py` | `/zed_camera_left/image_raw`、`/zed_camera_right/image_raw`(rgb8、1280x720、RELIABLE) |
| 內參/基線 | `config/isaac_zedx_stereo.yaml` | fx = fy = 490.667、cx = 640、cy = 360、無畸變;基線 0.11988 m(camera_info 的 P[3] 是 0,從 USD 量);演算法參數跟 `zed_stereo_vslam.yaml` 相同 |
| 相機 TF | `config/isaac_zedx_camera_tf.yaml` | 機器人 URDF 沒有相機 link,launch 補 `L_HEAD_J2 -> zed_left/right_camera_link -> zed_left/right_camera_frame`(光學座標系) |
| 本體遮罩 | `config/isaac_zedx_self_mask.png` | 相機看得到機器人自己的雙手,不遮 VSLAM 會以為相機沒在動;`scripts/make_self_mask.py` 產生 |
| ROS 參數 | `config/isaac_vslam_ros_params.yaml` | 模擬時間、地圖座標系 `vslam_map`(不搶 AMCL 的 `map -> odom`)、地圖原點貼地(`robot_base`)、`odom -> vslam_map` 靜態 TF、CLAHE、ExactTime 同步、視覺化 topic |
| DDS | `config/cyclonedds.xml` | 同一台主機 participant 多,參與者索引上限拉到 100(跟機器人端 devcontainer 設定一樣) |

### 輸出(給後續 SLAM / 定位行為用)

| topic / TF | 型別 | 內容 |
|---|---|---|
| `/run_slam/robot_pose` | `nav_msgs/Odometry` | 機器人底盤 `L_BASE_FOOTPRINT` 在 `vslam_map` 的位姿(`vslam_map` 原點 = 建圖當下的底盤,貼地) |
| `/run_slam/camera_pose` | `nav_msgs/Odometry` | 左相機 `zed_left_camera_link` 在 `vslam_map` 的位姿 |
| `/run_slam/keyframes`、`/run_slam/keyframes_2d` | `geometry_msgs/PoseArray` | 關鍵幀位姿(依關鍵幀 ID 排序,連起來就是關鍵幀軌跡) |
| `/run_slam/tracking_state` | `std_msgs/String`(transient local) | `Initializing` / `Tracking` / `Lost`,狀態改變時發 |
| `/run_slam/tracking_image`(`/compressed`) | `sensor_msgs/Image`(`CompressedImage`) | 原始左影像畫上特徵點:綠 = 對到地圖點,橘 = 沒對到,暗區 = 遮罩;每 2 幀一張,`/compressed` 是 JPEG |
| `/run_slam/frame_points` | `sensor_msgs/PointCloud2` | 這一幀的特徵點用雙目深度轉 3D、再用估計位姿放進 `vslam_map`(`tracked` = 1 是對到地圖點的) |
| `/run_slam/map_points` | `sensor_msgs/PointCloud2` | 全部地圖點,每 10 幀一次 |
| `/run_slam/loop_edges` | `visualization_msgs/Marker` | 迴環邊(兩端關鍵幀的相機位置) |
| TF `odom -> vslam_map` | 靜態 | 地圖第一筆位姿時,讓 VSLAM 的底盤位姿跟 odom 的重合;之後兩者的差就是 VSLAM 相對 odom 的漂移 |

### 驗證工具(`ros2 run vslam_bringup <工具>`,容器裡用 `./ros2_container.sh run ros2 run ...`)

| 工具 | 用途 |
|---|---|
| `camera_info_to_stella_yaml.py` | 從左右 camera_info(+ TF 取基線)印出 stella_vslam 的 Camera 區塊;`-p check:=<yaml>` 比對設定檔,不一致 exit 1。換解析度/場景時先跑這個 |
| `make_self_mask.py` | 用雙目視差找「一直在 1 m 內」的像素(機器人本體)產生遮罩 PNG,另存預覽圖 |
| `drive_waypoints.py` | 沿路網節點原地轉向 + 直線開(直接發 `/cmd_vel`,`/odom` 回授),預設繞 a7-a6-a5-a8 矩形一圈 |
| `vslam_trajectory_recorder.py` | 同時間戳記錄 VSLAM 位姿跟真值(Isaac `/odom` × 相機外參),輸出兩份 TUM 檔 |
| `evaluate_trajectory.py` | ATE(SE3 對齊)、Sim3 尺度(雙目正確應接近 1)、起點對齊的終點漂移,輸出疊圖;只需要 numpy |
| `vslam_viz_recorder.py` | 錄影片用的原始資料:追蹤影像(JPEG)、這一幀 3D 特徵點、地圖點、關鍵幀、迴環邊、VSLAM 位姿、真值、TF、2D 佔據地圖 |
| `render_vslam_video.py` | 把上面錄的資料離線做成 `camera.mp4`(相機視角 + 特徵點)、`topdown.mp4`(俯視全域地圖)、`side_by_side.mp4`;不需要 ROS |

驗證流程(VSLAM 已經在跑):

```bash
# 終端機 A:記錄(datasets/ 不進版控)
./ros2_container.sh run ros2 run vslam_bringup vslam_trajectory_recorder.py --ros-args \
  -p output_dir:=/workspaces/VSLAM_package/datasets/isaac_eval/mytest -p use_sim_time:=true
# 終端機 B:繞一圈,跑完回終端機 A 按 Ctrl+C
./ros2_container.sh run ros2 run vslam_bringup drive_waypoints.py
# 算誤差
./ros2_container.sh run bash -c "cd datasets/isaac_eval/mytest && ros2 run vslam_bringup evaluate_trajectory.py gt_tum.txt est_tum.txt --plot eval.png"
```

### 錄影片(相機視角特徵點 + 俯視全域地圖)

```bash
# 終端機 A:錄(VSLAM 啟動前後都可以;建圖時 launch 帶 map_db_out,純定位時帶 map_db_in + disable_mapping)
./ros2_container.sh run ros2 run vslam_bringup vslam_viz_recorder.py --ros-args \
  -p output_dir:=/workspaces/VSLAM_package/datasets/isaac_eval/myvideo -p use_sim_time:=true
# 終端機 B:./ros2_container.sh launch ...、drive_waypoints.py 繞一圈,跑完回終端機 A 按 Ctrl+C
# 渲染(容器裡有 H.264 編碼器跟掛進去的中文字型),輸出在 <錄製目錄>/videos/
./ros2_container.sh run ros2 run vslam_bringup render_vslam_video.py datasets/isaac_eval/myvideo --mode mapping
./ros2_container.sh run ros2 run vslam_bringup render_vslam_video.py datasets/isaac_eval/myloc --mode localization \
  --anchor-tf datasets/isaac_eval/myvideo/tf.txt     # 用建圖那次的 odom -> vslam_map,誤差才是相對地圖的絕對誤差
```

- 時間軸是模擬時間,30 fps = 機器人實際速度。俯視範圍預設是軌跡外框往外 2.5 m(`--view-margin`、`--view` 可改)。
- 底圖是機器人的 2D 佔據地圖,用錄到的 AMCL `map -> odom` 擺;AMCL 沒定位好時加 `--map-to-odom identity`
  (把 odom 當成地圖座標系)。這時巡航也別用 AMCL:`drive_waypoints.py -p map_frame:=odom`。
- 容器要有 `/usr/share/fonts` 的掛載才有中文字(`ros2_container.sh up` 建的容器已經掛了;舊容器 `down` 再 `up`)。

### 驗證結果

同一條路線(約 12.7–13.4 m、4 個 90° 原地轉向)的即時位姿:

| 設定 | 結果 |
|---|---|
| 不遮罩、不做 CLAHE | 被雙手特徵拉住,估計軌跡只在起點 0.4 m 內;Sim3 尺度 2.88、ATE 1.46 m |
| 遮罩 | 追蹤時 ATE 4.3 cm、尺度 1.07,但 8.8 s 後在素色牆前 tracking lost,之後沒救回 |
| 遮罩 + CLAHE(= 現在的設定,跑兩次) | 兩次都整圈沒有 lost、回到起點偵測到 loop;ATE 4.6 / 17.1 cm、Sim3 尺度 1.003 / 1.017、只用起點對齊的終點誤差 3.9 / 1.0 cm;追蹤耗時中位數約 37 ms |
| 載入上面建的地圖只定位 | 第一幀就重定位成功、整圈沒有 lost;相對建圖座標系的絕對誤差 RMSE 5.0 cm(起點 3.7 cm、終點 3.6 cm),平面牆那段也只差 1 cm |
| 2026-10-01 錄影那兩次(run9 建圖、run10 定位) | 建圖 ATE 17.3 cm、尺度 1.005、回到起點有迴環、終點 1.1 cm;地圖修正後平面牆那段仍有變形(關鍵幀 ATE 13.0 cm),定位時誤差中位數 2.7 cm,但在那段最大 70 cm(a5 轉角短暫 lost 一次後重定位到變形的那段) |

尺度接近 1 代表內參跟基線對。實際使用建議先繞一圈建圖(回到起點觸發 loop closure)存檔,之後載入地圖定位。兩次 ATE 差很多,誤差幾乎都來自 a6 -> a5 那段正對一整面平面素色牆的路(視覺退化,
會少估前進量),細節、試過的參數(`depth_threshold`、輪式里程計運動先驗都沒有可量測的改善)跟其他限制見
`../docs/isaac_sim_integration.md`。

## 實際接 ZED 相機運作前,還有 3 個地方要填

編譯、部署機制都已經驗證過沒問題,但不是空的就能直接接真的相機用——這 3 個檔案裡的值目前是佔位符或
未驗證的猜測,要實際有相機/`zed-ros2-wrapper` 後才能填真的:

| 檔案 | 現在的狀態 | 你要做的事 |
|---|---|---|
| `config/zed_stereo_vslam.yaml` | `fx`/`fy`/`cx`/`cy`/`focal_x_baseline`/`cols`/`rows`/`fps` 全部是佔位符(標 `# TODO`) | 拿到實體 ZED 後,照 `../docs/zed_stereo.md` 步驟填入真實校正值 |
| `launch/stereo_vslam.launch.py` 的 `left_image_topic`/`right_image_topic` 預設值 | 常見 `zed-ros2-wrapper` 命名慣例,**沒有實際裝過 zed-ros2-wrapper 驗證過** | 裝好 `zed-ros2-wrapper` 後,自己 `ros2 topic list` 核對真實 topic 名稱 |
| `config/vslam_ros_params.yaml` 的 `odom_frame`/`map_frame`/`robot_base_frame` | ROS2 常見慣例值,不是照實際機器人 TF 樹填的 | 真的要接上機器人平台時,跟對方的 TF 樹核對 |

## 為什麼會有這個套件(跟 stella_vslam_ros 差在哪)

`stella_vslam_ros` 是**別人的 open source 套件**(用 git submodule 引用,見主 `README.md`「版控說明」),
裡面的 `run_slam` 節點寫死訂閱 `camera/left/image_raw`、`camera/right/image_raw` 這種通用 topic 名稱,
也不帶任何 launch 檔——直接用需要自己每次手動組一長串 `ros2 run` 參數。

`vslam_bringup` 是**你自己的部署設定**,不是第三方 repo,所以直接放在 `VSLAM_package` 自己的版控裡
(不是 submodule),裡面放:

- `launch/stereo_vslam.launch.py`——一個指令啟動,自動帶好參數、把 topic remap 到實際的 ZED topic
- `config/zed_stereo_vslam.yaml`——ZED 相機校正參數 + stella_vslam 演算法參數(對應本地端測試用的
  `../configs/ZED_stereo.yaml`,但這是給 ROS2 部署用的獨立副本,原因見檔案開頭註解)
- `config/vslam_ros_params.yaml`——ROS2 節點參數(TF frame id、是否發布 TF 等)
- `vocab/orb_vocab.fbow`——ORB 詞彙檔,不進版控(43MB,見 `.gitignore`),`../colcon_build.sh` 會在
  build 前自動準備好(複製本地端測試已下載的那份,沒有就現抓),讓它跟著這個套件一起裝進 ROS2 的
  share 目錄,launch 檔預設路徑才不會綁死特定機器的絕對路徑

## 前提

1. **有 ROS2 環境,`stella_vslam_ros` 的依賴要補齊**。`VSLAM_package` 根目錄現在已經是一個 colcon
   workspace(`src/` 底下是指回 `g2o`/`stella_vslam`/`stella_vslam_ros`/`vslam_bringup` 的 symlink),
   不用自己手動決定 workspace 位置或設 `CMAKE_PREFIX_PATH`——`../colcon_build.sh` 一個指令會把
   `g2o → stella_vslam → stella_vslam_ros → vslam_bringup` 依序編完(細節、取捨說明見主 `README.md`
   「未來部署方式:ROS2」章節)。唯一要自己先做的是把 `stella_vslam_ros` 列的 ROS2 依賴(`rclcpp`、
   `cv_bridge`、`tf2` 全家桶等,完整清單見 `../stella_vslam_ros/package.xml`)裝齊,建議在容器裡跑
   `rosdep install --from-paths ../src --ignore-src -r -y`。
2. **要有 ZED 影像來源**——目前規劃是用 Stereolabs 官方
   [`zed-ros2-wrapper`](https://github.com/stereolabs/zed-ros2-wrapper),發布已經 rectify 過的左右
   影像 topic(見 `../docs/zed_stereo.md` 的建議路)。裝好之後要核對的細節見下面「實際接 ZED 相機運作
   前」那張表。

## 用法(等前提都滿足之後)

```bash
# 在 VSLAM_package 根目錄(不是這裡)
cd ..
source /opt/ros/<distro>/setup.bash
./colcon_build.sh              # 第一次,或 g2o/stella_vslam 有改動時
source install/setup.bash

ros2 launch vslam_bringup stereo_vslam.launch.py \
  left_image_topic:=/zed/zed_node/left/image_rect_color \
  right_image_topic:=/zed/zed_node/right/image_rect_color
```

只改了 `stella_vslam_ros`/`vslam_bringup` 這兩塊(沒動 `g2o`/`stella_vslam`)的話,不用整個重編,
用 `colcon build --base-paths src --packages-select stella_vslam_ros vslam_bringup` 加減省點時間。

常用覆寫參數(完整清單見 `launch/stereo_vslam.launch.py` 裡每個 `DeclareLaunchArgument` 的說明):

| 參數 | 預設值 | 用途 |
|---|---|---|
| `vocab_file` | 這個套件 share 目錄底下的 `vocab/orb_vocab.fbow`(`colcon_build.sh` 自動準備,不綁死機器路徑) | ORB 詞彙檔路徑 |
| `config_file` | `config/zed_stereo_vslam.yaml` | 相機/演算法設定 |
| `params_file` | `config/vslam_ros_params.yaml` | ROS2 節點參數(TF 等) |
| `viewer` | `none` | 機上無頭部署預設不開視窗;要現場除錯看畫面才改 |
| `left_image_topic` / `right_image_topic` | ZED wrapper 常見預設(未驗證) | 實際 ZED 左右眼 topic,務必自行核對 |

節點啟動後可以用的輸出(topic 名稱前面會自動加節點名稱 `run_slam`,即 ROS2 的 private topic 慣例):

| topic | 型別 | 內容 |
|---|---|---|
| `/run_slam/camera_pose` | `nav_msgs/Odometry` | 目前估計的相機位姿(map_frame → camera_frame) |
| `/run_slam/keyframes` | `geometry_msgs/PoseArray` | 所有關鍵幀的 3D 位置,拿來在 RViz 看地圖建構狀況 |
| `/run_slam/keyframes_2d` | `geometry_msgs/PoseArray` | 同上,投影到 2D 平面版本 |

如果 `vslam_ros_params.yaml` 裡 `publish_tf: true`,還會發布 `map` → `odom` 的 TF(需要另外有節點在發布
`odom` → `base_link`,例如輪速計,`stella_vslam_ros` 不會自己生出這段)。

fork 版 `stella_vslam_ros` 另外加了 `/run_slam/robot_pose`、`/run_slam/tracking_state`、`/run_slam/tracking_image`
跟 `map_frame_origin`、`map_parent_frame`、`clahe_clip_limit` 等參數(預設值維持上游行為,說明見
`../stella_vslam_ros/README.md`),實體 ZED 要用的話參考 `config/isaac_vslam_ros_params.yaml` 的寫法。

## 已知限制 / 待辦

- [ ] 上面「實際接 ZED 相機運作前」那 3 個地方(校正參數、topic 名稱、TF frame 名稱)都還沒有實體
      相機/`zed-ros2-wrapper` 可以核對
- [ ] Isaac Sim:相機 TF 是 launch 用靜態 TF 補的,機器人 URDF(`amr_base_sw/urdf/TOTAL_AMR_V1.5_FIH_Hand_0.1t_0.1t.urdf`)
      加上 ZED X link 之後改 `publish_camera_tf:=false`
- [ ] Isaac Sim:本體遮罩只對目前的手臂姿勢有效,手臂動了要用 `make_self_mask.py` 重拍
- [ ] Isaac Sim:要讓 VSLAM 接手 `map -> odom`,機器人端要有「不跑 AMCL / slam_toolbox」的啟動選項,
      目前的 `start_sim_and_spawn_IsaacSim.launch.py` 兩種模式都會發 `map -> odom`
- [ ] 還沒決定要不要把 `zed-ros2-wrapper` 也用同一種 symlink 方式併進 `../src/`,或是併入
      `ros2-web-app` 既有 workspace(只有紙上規劃,見主 `README.md`「未來部署方式」章節)
