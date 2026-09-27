import os
from plantuml import PlantUML

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")

ARCHITECTURE = r"""
@startuml
' --- 樣式精準控制 ---
skinparam RectangleWidthMax 380
skinparam ranksep 55
skinparam nodesep 70
skinparam shadowing false
skinparam defaultFontName "Microsoft JhengHei"

title VSLAM_package 核心架構：四個套件的分工（colcon 編譯順序 g2o → stella_vslam → stella_vslam_ros → vslam_bringup）

rectangle Camera #F5F5F5 [
  **ZED 雙目相機（zed-ros2-wrapper，尚未安裝）**
  * 發布已校正 (rectified) 的左右影像 topic。
  * 預設假設 /zed/zed_node/left(right)/image_rect_color
    （未實測，接上相機後要用 ros2 topic list 核對）。
]

rectangle Bringup #D1E5F0 [
  **vslam_bringup（自己寫的部署套件）**
  * launch/stereo_vslam.launch.py：一個指令啟動 run_slam。
  * 把 ZED topic remap 到 camera/left(right)/image_raw。
  * config/zed_stereo_vslam.yaml：相機校正＋演算法參數
    （校正值目前仍是 # TODO 佔位符）。
  * config/vslam_ros_params.yaml：TF frame、publish_tf 等節點參數。
  * vocab/orb_vocab.fbow：隨套件安裝，不綁機器絕對路徑。
]

rectangle RosNode #D5E8D4 [
  **stella_vslam_ros（ROS2 包裝層，run_slam 節點）**
  * message_filters 同步左右影像（Approximate/Exact Time）。
  * cv_bridge 轉成 cv::Mat，丟給 feed_stereo_frame()。
  * 發布 ~/camera_pose、~/keyframes、~/keyframes_2d，
    publish_tf=true 時發布 map→odom TF。
  * 訂閱 /initialpose，可手動指定位姿做重定位。
]

rectangle Core #FFF2CC [
  **stella_vslam（核心演算法庫）**
  * Tracking（主執行緒）：ORB 特徵＋立體匹配、逐幀位姿估計。
  * Mapping（背景執行緒）：三角化、融合、local BA、關鍵幀清理。
  * Global Optimization（背景執行緒）：FBoW 迴圈偵測、
    Sim3 驗證、Pose Graph、全域 BA。
  * map_database：關鍵幀/地標/共視圖，可存成 msgpack。
]

rectangle G2O #E1D5E7 [
  **g2o（圖優化後端）**
  * 所有最佳化都在這裡解：單幀 pose optimization、
    local BA、pose graph、全域 BA。
  * 自己的 fork（DavidZox/g2o，vslam-deploy 分支），
    修過 Config.cmake.in 的 OpenGL 依賴 bug。
]

rectangle Downstream #ECECEC [
  **下游（規劃中）**
  * RViz 檢視關鍵幀/軌跡。
  * 導航/機器人平台讀 TF 與 camera_pose
    （需另有節點發布 odom→base_link）。
]

Camera -down-> Bringup : 影像 topic
Bringup -down-> RosNode : 啟動節點＋參數＋topic remap
RosNode -down-> Core : feed_stereo_frame(left, right, timestamp)
Core -down-> G2O : 最佳化問題
Core -[#009933,bold]up-> RosNode : <color:#000>**相機位姿 / 關鍵幀**</color>
RosNode -[#CC6600,bold]right-> Downstream : <color:#000>**Odometry / PoseArray / TF**</color>

note right of Bringup
  **只有這個套件是自己的程式碼**，
  其他三個都是上游 open source
  的 fork（git submodule），
  改演算法才需要進去改。
end note

note right of Core
  **三條執行緒：**
  Tracking 每幀都跑、必須即時；
  Mapping 只處理被選中的關鍵幀；
  Global Optimization 只在
  偵測到迴圈時才做重活。
end note

note left of RosNode
  **-r 撞名 bug 已修：**
  原本自訂的 -r/--rectify 會吃掉
  ROS2 的 -r（topic remap），
  已拿掉短參數，只留 --rectify。
end note

@enduml
"""

STATE_MACHINE = r"""
@startuml
' --- 樣式精準控制 ---
skinparam RectangleWidthMax 360
skinparam ranksep 55
skinparam nodesep 65
skinparam shadowing false
skinparam defaultFontName "Microsoft JhengHei"

title 追蹤模組 (tracking_module) 狀態機：每一幀進來依 tracker_state_t 決定做什麼

rectangle State1 #D1E5F0 [
  **1. Initializing（初始化）**
  * 系統剛啟動，或被 reset 之後的狀態。
  * 雙目：只要這一幀有效深度的特徵點數
    ≥ min_num_triangulated_pts（ZED 設定 100），
    就直接用這一幀建出第一個關鍵幀＋初始地標。
  * 不像單目需要等兩幀有足夠視差。
]

rectangle State2 #D5E8D4 [
  **2. Tracking（正常追蹤）**
  * 先用等速度運動模型投影匹配，失敗改用 BoW 匹配。
  * track_local_map：對照共視關鍵幀的地標再精修位姿。
  * 成功且符合條件 → 插入新關鍵幀交給 Mapping。
  * **唯一會輸出位姿、會新增關鍵幀的狀態。**
]

rectangle State3 #F8CECC [
  **3. Lost（追蹤丟失）**
  * 這一幀追蹤失敗（匹配點不足等）。
  * 下一幀開始改走 relocalizer：
    FBoW 找候選關鍵幀 → PnP + RANSAC → g2o 精修。
  * 丟失期間不輸出位姿、不新增關鍵幀。
]

rectangle Reset #FFF2CC [
  **系統重置 (reset)**
  * 清空地圖、關鍵幀、BoW 資料庫。
  * 回到 Initializing 重新建圖。
]

State1 -[#009933,bold]down-> State2 : 有效深度點數足夠\n(初始化成功)
State1 -right-> State1 : 點數不足，下一幀再試

State2 -right-> State2 : 追蹤成功
State2 -[#CC0000,bold]down-> State3 : **追蹤失敗**

State3 -[#009933,bold]up-> State2 : <color:#000>**重定位成功**</color>\n(自動 relocalizer 或 /initialpose)
State3 -down-> State3 : 重定位失敗，下一幀再試

State2 -[#CC6600,bold]left-> Reset : <color:#000>**初始化後 5 秒內就丟失**</color>\n(init_retry_threshold_time)
Reset -up-> State1

note right of State3
  **兩種重定位方式：**
  1. 自動：enable_auto_relocalization
     （預設開），每幀都試。
  2. 手動：對 /initialpose 發一個
     PoseWithCovarianceStamped，
     系統會找附近的關鍵幀對齊。
end note

note left of Reset
  **為什麼要重置：**
  剛初始化就丟失，代表初始地圖
  品質太差，與其硬救不如重來；
  超過 5 秒才丟失就改走重定位，
  保留已經建好的地圖。
end note

note bottom of State3
  **定位模式（--disable-mapping）**
  狀態機一樣，差別只在 Tracking
  不會再插入新關鍵幀，Lost 後
  完全靠載入地圖的關鍵幀重定位。
end note

@enduml
"""

PIPELINE = r"""
@startuml
' --- 樣式精準控制 ---
skinparam shadowing false
skinparam defaultFontName "Microsoft JhengHei"
skinparam ActivityBackgroundColor #FFF8E7
skinparam ActivityBorderColor #996633
skinparam NoteBackgroundColor #FFFFE0

title VSLAM 資料處理管線：前景每幀追蹤要即時，建圖與全域優化丟到背景執行緒慢慢做

partition "啟動時（ros2 launch vslam_bringup stereo_vslam.launch.py）" {
start
:launch 檔組好參數，啟動 **run_slam**
-v 詞彙檔、-c zed_stereo_vslam.yaml、--viewer none
topic remap 到 ZED 左右影像;
:**① 讀 config**（stella_vslam::config）
Camera / Feature / Mapping / Tracking 等區塊; <<#FCE5CD>>
:**② 建立 stella_vslam::system**
載入 ORB 詞彙檔 orb_vocab.fbow（FBoW）
建立左右兩個 orb_extractor; <<#FCE5CD>>
if (有給 -i / --map-db-in？) then (有)
  :load_map_database() 載入既有地圖
  （不需要重新初始化）;
else (沒有)
  :從空地圖開始，等第一幀初始化;
endif
:**③ system::startup()**
另開兩條背景執行緒：Mapping、Global Optimization; <<#FCE5CD>>
if (有加 --disable-mapping？) then (有)
  :關閉 Mapping（定位模式，地圖只讀不寫）;
endif
:rclcpp::spin() 開始等影像;
stop
}

partition "每收到一組左右影像（stereo::callback）" {
start
:message_filters 同步左右影像
（use_exact_time=false → ApproximateTime）;
:cv_bridge 轉成 cv::Mat;
if (有開 --rectify？) then (有)
  :StereoRectifier 校正
  （ZED 已輸出校正影像，預設不開）;
endif
:**④ feed_stereo_frame()**
左右影像各開一條執行緒跑 ORB 擷取
→ match::stereo 立體匹配算出每個特徵點深度; <<#FCE5CD>>
:**⑤ tracking_module::feed_frame()**
依狀態機：初始化 / 追蹤 / 重定位
g2o 做單幀 pose optimization; <<#FCE5CD>>
if (追蹤成功且需要新關鍵幀？) then (是)
  :keyframe_inserter 建立關鍵幀
  丟進 Mapping 的佇列;
endif
:**⑥ 發布結果**
~/camera_pose（Odometry）
publish_tf=true → map→odom TF
publish_keyframes=true → ~/keyframes、~/keyframes_2d; <<#FCE5CD>>
stop
}

partition "Mapping 背景執行緒（每個新關鍵幀跑一次）" {
start
:store_new_keyframe：關鍵幀寫入 map_database;
:remove_invalid_landmarks：清掉品質不好的新地標;
:**create_new_landmarks**
跟前 N 個共視關鍵幀三角化新地標
（num_covisibilities_for_landmark_generation）; <<#FCE5CD>>
:**update_new_keyframe**
跟前 N 個共視關鍵幀融合重複地標
（num_covisibilities_for_landmark_fusion）; <<#FCE5CD>>
if (佇列積太多關鍵幀？) then (是)
  :跳過 local BA（算力不夠時優先保即時）;
else (否)
  :**local BA（g2o）**
  同時優化附近關鍵幀位姿＋地標座標; <<#FCE5CD>>
endif
:remove_redundant_keyframes
（redundant_obs_ratio_thr）;
:queue_keyframe → 交給 Global Optimization;
stop
}

partition "Global Optimization 背景執行緒" {
start
:從佇列取出關鍵幀，加入 BoW 資料庫;
if (detect_loop_candidates：FBoW 找到相似關鍵幀？) then (沒有)
  :沒有迴圈，等下一個關鍵幀;
  stop
else (有)
endif
if (validate_candidates：Sim3 幾何驗證通過？) then (不通過)
  :當作誤判，捨棄;
  stop
else (通過)
endif
:**correct_loop**
暫停 Mapping → Sim3 修正共視關鍵幀/地標
→ **Pose Graph Optimization**（本質圖，g2o）; <<#FCE5CD>>
:另開執行緒跑 **全域 BA**（loop_bundle_adjuster）
整張地圖再精修一次;
note right
  全域 BA 最慢但最準，
  跑在背景不擋前景追蹤；
  中途有新迴圈會中止重跑
end note
stop
}

partition "關閉時" {
start
:Ctrl+C → rclcpp::spin() 返回;
:SLAM->shutdown()：停止背景執行緒;
if (有給 -o / --map-db-out？) then (有)
  :save_map_database() 存成 msgpack
  （下次可用 -i 載入做定位）;
endif
stop
}

@enduml
"""


def render(puml_syntax: str, file_base: str):
    server = PlantUML(url='http://www.plantuml.com/plantuml/img/')
    os.makedirs(OUT_DIR, exist_ok=True)
    puml_path = os.path.join(OUT_DIR, f"{file_base}.puml")
    png_path = os.path.join(OUT_DIR, f"{file_base}.png")

    try:
        with open(puml_path, "w", encoding="utf-8") as f:
            f.write(puml_syntax.strip() + "\n")
        print(f"✅ 語法檔儲存成功：{puml_path}")
    except Exception as e:
        print(f"❌ 語法檔儲存失敗: {e}")

    try:
        raw_image_data = server.processes(puml_syntax)
        with open(png_path, 'wb') as f:
            f.write(raw_image_data)
        print(f"✅ 圖片檔轉檔成功：{png_path}")
    except Exception as e:
        print(f"❌ 圖片轉檔失敗 (請檢查網路): {e}")


if __name__ == "__main__":
    render(ARCHITECTURE, "VSLAM核心架構說明")
    render(STATE_MACHINE, "追蹤模組的狀態轉移")
    render(PIPELINE, "VSLAM資料處理管線")
