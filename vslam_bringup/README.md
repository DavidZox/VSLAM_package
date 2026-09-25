# vslam_bringup

雙目(ZED)VSLAM 部署用的 ROS2 **bringup 套件**——只包 launch 檔跟參數 YAML,**不含 SLAM 演算法本身**。
實際運算由 [`stella_vslam_ros`](../stella_vslam_ros/)(submodule)提供,這個套件單純負責「組裝參數、
把相機 topic 接起來、設定 TF」,跟 `ros2-web-app` 那類專案裡常見的 `*_bringup` 套件角色一樣。

✅ **已經在真正的 ROS2 Jazzy 環境驗證過**:`colcon_build.sh` 在一個實際跑著的 ROS2 Jazzy 容器裡從乾淨
狀態跑過,`g2o`/`stella_vslam`/`stella_vslam_ros`/`vslam_bringup` 四個套件全部編譯成功,
`ros2 launch vslam_bringup stereo_vslam.launch.py --show-args` 也確認 launch 檔跟參數都正常。過程中
連帶修掉 `stella_vslam_ros` 兩個真實 bug(ament 環境 hook 重複註冊、`cv_bridge.h`→`.hpp` 改名),細節
見主 `README.md`「已知眉角」。

還沒驗證的只剩「接上真的 ZED 相機」這塊——編譯跟 launch 檔本身沒問題,但實際影像 topic 名稱、TF 設定
都還是紙上規劃,要等實體相機/`zed-ros2-wrapper` 到位才能確認,具體是下面這 3 個地方:

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

## 已知限制 / 待辦

- [ ] 上面「實際接 ZED 相機運作前」那 3 個地方(校正參數、topic 名稱、TF frame 名稱)都還沒有實體
      相機/`zed-ros2-wrapper` 可以核對
- [ ] 還沒決定要不要把 `zed-ros2-wrapper` 也用同一種 symlink 方式併進 `../src/`,或是併入
      `ros2-web-app` 既有 workspace(只有紙上規劃,見主 `README.md`「未來部署方式」章節)
