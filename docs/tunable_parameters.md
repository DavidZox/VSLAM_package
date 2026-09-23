# stella_vslam 全流程可調參數對照表

目標:把整條 pipeline(前端特徵擷取 → 初始化 → 追蹤 → 建圖/local BA → 迴環偵測 → global BA/pose graph)
裡所有能調的參數都列出來,包含官方範例 YAML 沒列出來、但程式碼裡其實吃得到的「隱藏」參數,方便系統性
地一個一個試、看結果差異、理解演算法。

共約 105 個 YAML 參數。「範例?」欄位:**Y**=大多數官方範例都有設、**P**=只有少數範例有設(通常只有
`stella_vslam/example/aist/*.yaml`)、**N**=隱藏參數,官方範例都沒設,只能查程式碼才知道存在、目前吃的
是程式碼內建的預設值。

相機校正參數(`Camera:` 的 fx/fy/cx/cy/畸變、`StereoRectifier:`)不算演算法調參,這裡不重複列,已經在
主 README 跟 `configs/ZED_stereo.yaml` 說明過。

## ⚠️ 先看這個:官方範例 YAML 裡 `num_grid_cols`/`num_grid_rows` 其實是壞的

程式碼是從 `Preprocessing.num_grid_cols` / `Preprocessing.num_grid_rows` 讀值
(`src/stella_vslam/system.cc:112-113`,預設 64/48)。但**官方所有範例 YAML**(KITTI、EuRoC、TUM_VI、
aist)都是設在 `System:` 底下,不是 `Preprocessing:`——`util::yaml_optional_ref` 只會把
`preprocessing_params` 綁定到 `Preprocessing` 這個子節點,放在 `System:` 底下的值它看不到,所以**這些
範例 YAML 裡設的 grid 大小(例如 KITTI 的 40/13、TUM_VI 的 16/16)實際上從來沒生效過**,一直都是悄悄
用預設值 64/48。用 `git blame` 確認過,這個參數從 2024/4 引入(`e6ffa25`)那次 commit 就已經是
「程式碼跟範例不一致」的狀態,到現在 HEAD 都還沒修。**如果你想真的調 grid 大小,要把這兩個 key 放到
`Preprocessing:` 底下,不是 `System:`。**這也是很好的「深入了解演算法」案例——連官方範例都會有這種
沒人發現的設定失效問題。

---

## 1. 前端 — 特徵擷取 / 前處理

原始碼:`feature/orb_params.{h,cc}`、`feature/orb_extractor.{h,cc}`、`system.cc`

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `Feature.name` | 這組 ORB 參數的識別字串,純標籤無數值效果 | `"default ORB feature extraction setting"` | Y |
| `Feature.scale_factor` | 金字塔相鄰層級的縮放比例 | `1.2` | Y |
| `Feature.num_levels` | 金字塔層數(決定尺度不變範圍) | `8` | Y |
| `Feature.ini_fast_threshold` | 每個 grid cell 優先嘗試的 FAST 角點響應閾值 | `20` | Y |
| `Feature.min_fast_threshold` | 若 `ini_fast_threshold` 在某 cell 找不到任何點,退而求其次用這個較寬鬆閾值 | `7` | Y |
| `Preprocessing.min_size` | 每個保留特徵點要分配到的目標像素**面積**(內部開根號成邊長),用來決定 quadtree/grid 的大小以做均勻空間分布——影響特徵點密度/數量,**不是**縮放影像尺寸 | `800` | Y |
| `Preprocessing.mask_rectangles` | FAST 偵測前要遮蔽的矩形區域列表,格式 `[x_min/cols, x_max/cols, y_min/rows, y_max/rows]` | `[]`(無) | P(只有 aist 全景範例) |
| `Preprocessing.descriptor_type` | 描述子後端:`"ORB"`、`"HASH_SIFT"`(需要 CUDA 編譯)、`"LIFTFEAT"`(需要 ONNX Runtime 編譯) | `"ORB"` | **N** |
| `Preprocessing.onnx_model_path` | ONNX 模型檔路徑,只有 `descriptor_type: LIFTFEAT` 時才會讀 | `"liftfeat_core.onnx"` | **N** |
| `Preprocessing.num_grid_cols` | 特徵點分配 grid 的欄數(`assign_keypoints_to_grid`,配對時用來快速空間查找)——見上面的 bug 說明 | `64` | **N**(範例有設,但設錯區塊) |
| `Preprocessing.num_grid_rows` | 同上,列數 | `48` | **N**(同上) |
| `Preprocessing.depthmap_factor` | RGBD 深度圖的縮放係數(`depth_raw / factor = 公尺深度`),只有 RGBD 模式才用 | `1.0` | P(只有 RGBD 範例) |

**完全沒開放 YAML 調整、寫死在 `orb_extractor.cc` 裡的**:FAST 搜尋格子大小(19px patch 半徑、
`cell_size=64`、`overlap=6` px)、描述子計算前的高斯模糊核 `7x7, σ=2`、LiftFeat 偵測閾值 `0.015`、
最大特徵點數 `4096`。

---

## 2. 初始化

原始碼:`module/initializer.{h,cc}`(YAML 區塊 `Initializer:`)

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `Initializer.num_ransac_iterations` | 單目初始化(E/H matrix)RANSAC 最大迭代次數 | `100` | **N** |
| `Initializer.min_num_valid_pts` | 嘗試幾何初始化前,兩張初始 frame 間最少要有幾個 area-match | `50` | **N** |
| `Initializer.min_num_triangulated_pts` | 接受初始化所需的最少三角化點數(單目/雙目/RGBD 都適用) | `50` | Y |
| `Initializer.parallax_deg_threshold` | 單目初始化所需的最小視差角度(度),用來排除退化的純旋轉情況 | `1.0` | **N** |
| `Initializer.reprojection_error_threshold` | RANSAC 模型評分用的重投影誤差內點閾值(像素) | `4.0` | **N** |
| `Initializer.num_ba_iterations` | 初始兩張關鍵幀 global BA 的迭代次數 | `100` | **N** |
| `Initializer.scaling_factor` | 初始地圖正規化到「中位深度 = 1.0」之後,再乘上的額外縮放係數 | `1.0` | P |
| `Initializer.use_fixed_seed` | RANSAC 用固定亂數種子(可重現) | `false` | **N** |
| `Initializer.gain_threshold` | 初始化 BA 的 g2o optimizer gain 終止閾值 | `1e-5` | **N** |
| `Initializer.verbose` | 初始化 BA 的 g2o 詳細記錄 | `false` | **N** |

---

## 3. 追蹤

原始碼:`tracking_module.{h,cc}`(YAML 區塊 `Tracking:`)、`optimize/pose_optimizer_factory.h`

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `Tracking.enable_temporal_keyframe_only_tracking` | 若為 true,只對「temporal」關鍵幀追蹤時不算 Lost | `false` | Y |
| `Tracking.margin_last_frame_projection` | 用運動模型投影上一幀地標時的搜尋視窗半徑(像素) | `20.0` | Y |
| `Tracking.margin_local_map_projection` | 正常狀態下,local map 投影配對的搜尋視窗半徑(像素) | `5.0` | Y |
| `Tracking.margin_local_map_projection_unstable` | 同上,但用於 relocalization 後緊接著的 2 幀(搜尋範圍放寬) | `20.0` | **N** |
| `Tracking.max_num_local_keyfrms` | 建立追蹤用 local map 時考慮的最大關鍵幀數 | `60` | **N** |
| `Tracking.enable_auto_relocalization` | 追蹤丟失時自動嘗試 BoW relocalization | `true` | **N** |
| `Tracking.use_robust_matcher_for_relocalization_request` | `relocalize_by_pose()` API 呼叫時改用強健(暴力比對)匹配器而非 BoW | `false` | **N** |
| `Tracking.reloc_distance_threshold` | `relocalize_by_pose`/`_2d` API 判定關鍵幀「夠近」的最大距離(公尺) | `0.2` | **N** |
| `Tracking.reloc_angle_threshold` | 同上,最大角度(弧度) | `0.45` | **N** |
| `Tracking.init_retry_threshold_time` | 若初始化後這麼多秒內追蹤就丟失,直接重置整個地圖而非嘗試 relocalize | `5.0` | **N** |
| `Tracking.backend` | 姿態優化後端:`"g2o"` 或 `"gtsam"`(gtsam 需要 `USE_GTSAM` 編譯選項) | `"g2o"` | Y |
| `Tracking.g2o.num_trials_robust` | 強健(排除離群值)姿態優化嘗試次數 | `2` | **N** |
| `Tracking.g2o.num_trials` | 非強健姿態優化嘗試次數 | `2` | **N** |
| `Tracking.g2o.num_each_iter` | 每次嘗試的 LM 迭代數 | `10` | **N** |
| `Tracking.gtsam.num_iter` | gtsam 後端最大 LM 迭代數 | `5` | **N** |
| `Tracking.gtsam.relative_error_tol` | LM 相對誤差收斂容忍值 | `1e-2` | **N** |
| `Tracking.gtsam.lambda_initial` | LM 初始阻尼係數 | `1e-5` | **N** |
| `Tracking.gtsam.lambda_upper_bound` | LM 最大阻尼係數 | `1e-2` | **N** |
| `Tracking.gtsam.enable_outlier_elimination` | LM 每輪之間排除離群觀測值 | `true` | **N** |
| `Tracking.gtsam.verbosity` | gtsam LM 詳細記錄層級 | `"SILENT"` | **N** |

*(寫死、沒開放 YAML 調整:姿態優化的離群值卡方閾值固定 `5.99146`(2 自由度/單目)、`7.81473`
(3 自由度/雙目),對應 95% 信心水準;`frame_tracker` 的匹配數門檻寫死 `10`,並非標頭檔預設的 20。)*

### 3a. Relocalizer(`Relocalizer:` 區塊,`module/relocalizer.{h,cc}`)

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `Relocalizer.search_neighbor` | 同時搜尋 reloc 候選關鍵幀的共視鄰居,找更多 2D-3D 配對 | `true` | Y |
| `Relocalizer.bow_match_lowe_ratio` | BoW-tree 配對的 Lowe ratio | `0.75` | **N** |
| `Relocalizer.proj_match_lowe_ratio` | 投影配對的 Lowe ratio | `0.9` | **N** |
| `Relocalizer.robust_match_lowe_ratio` | 暴力/強健配對的 Lowe ratio | `0.8` | **N** |
| `Relocalizer.min_num_bow_matches` | 保留一個 reloc 候選所需的最少 BoW 配對數 | `20` | **N** |
| `Relocalizer.min_num_valid_obs` | 姿態精化後所需的最少有效觀測數 | `50` | **N** |
| `Relocalizer.use_fixed_seed` | PnP solver 用固定 RANSAC 種子 | `false` | **N** |
| `Relocalizer.top_n_covisibilities_to_search` | `search_neighbor: true` 時搜尋的鄰居關鍵幀數 | `10` | **N** |
| `Relocalizer.num_common_words_thr_ratio` | 候選檢索用的 BoW 共同詞彙比例閾值 | `0.8` | **N** |
| `Relocalizer.max_num_ransac_iter` | EPnP 最大 RANSAC 迭代數 | `30` | **N** |
| `Relocalizer.max_num_local_keyfrms` | `refine_pose_by_local_map` 用的 local map 大小 | `60` | **N** |

### 3b. 關鍵幀插入(`KeyframeInserter:` 區塊,`module/keyframe_inserter.{h,cc}`)

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `KeyframeInserter.wait_for_local_bundle_adjustment` | 新關鍵幀的 local BA 做完前,追蹤執行緒暫停等待 | `false` | Y |
| `KeyframeInserter.max_interval` | 距上次插入關鍵幀超過幾秒就強制插入(`≤0` 停用) | `1.0` | **N** |
| `KeyframeInserter.min_interval` | 距上次插入關鍵幀不到幾秒不允許插入(`≤0` 停用) | `0.1` | **N** |
| `KeyframeInserter.max_distance` | 移動距離超過幾公尺就強制插入(`≤0` 停用) | `-1.0`(關) | **N** |
| `KeyframeInserter.min_distance` | 移動距離不到幾公尺不允許插入(`≤0` 停用) | `-1.0`(關) | **N** |
| `KeyframeInserter.lms_ratio_thr_almost_all_lms_are_tracked` | 相對參考關鍵幀,可靠地標比例超過此值就跳過插入(視角沒什麼變化) | `0.9` | **N** |
| `KeyframeInserter.lms_ratio_thr_view_changed` | 可靠地標比例低於此值就強制插入(視角變化很大)⚠️見下方註 | `0.5` | **N** |
| `KeyframeInserter.enough_lms_thr` | 可靠地標數低於此值就強制插入 | `100` | **N** |
| `KeyframeInserter.required_keyframes_for_marker_initialization` | ArUco 標記要被幾個關鍵幀觀測到才初始化其 3D 姿態 | `3` | **N** |

> 註:`lms_ratio_thr_view_changed` 的純 C++ 建構子預設是 `0.8`,但實際使用的 YAML 解析建構子預設是
> `0.5`——header 跟「省略這個 key 時實際拿到的值」不一致,是程式碼本身一個小小的不一致之處。

---

## 4. 建圖 / Local Bundle Adjustment

原始碼:`mapping_module.{h,cc}`(區塊 `Mapping:`)、`module/local_map_cleaner.{h,cc}`(同區塊)、
`optimize/local_bundle_adjuster_{g2o,gtsam}.cc`

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `Mapping.backend` | Local BA 後端:`"g2o"` 或 `"gtsam"` | `"g2o"` | Y |
| `Mapping.baseline_dist_thr` | 目前關鍵幀與共視關鍵幀之間,嘗試三角化所需的最小相機基線絕對距離(地圖單位)。跟 `_ratio` 互斥(兩個都設會丟例外) | `1.0` | Y |
| `Mapping.baseline_dist_thr_ratio` | 同上,但改用鄰居關鍵幀場景中位深度的比例表示(能自動適應尺度)——`baseline_dist_thr` 沒設時才用這個 | `0.02` | Y |
| `Mapping.redundant_obs_ratio_thr` | 關鍵幀剔除閾值:若某共視關鍵幀有這個比例以上的觀測是「冗餘」(被 ≥3 個其他關鍵幀以相同或更好尺度看到),就刪掉它。負值停用剔除 | `0.9` | Y |
| `Mapping.num_covisibilities_for_landmark_generation` | 每個新關鍵幀,檢查前 N 個共視關鍵幀來三角化新地標 | `10` | Y |
| `Mapping.num_covisibilities_for_landmark_fusion` | 每個新關鍵幀,檢查前 N 個共視關鍵幀來融合重複地標 | `10` | Y |
| `Mapping.erase_temporal_keyframes` | 是否刪除超過 `num_temporal_keyframes` 天數的「temporal」關鍵幀(用於定位模式工作流程) | `false` | Y |
| `Mapping.num_temporal_keyframes` | temporal 關鍵幀被刪除前的存活年齡(以關鍵幀 id 差計) | `15` | Y |
| `Mapping.residual_deg_thr` | 新地標三角化搜尋時,接受候選配對的極線殘差角度閾值(度,內部轉弧度) | `0.2` | P(只有 aist,設 0.4) |
| `Mapping.observed_ratio_thr` | 地標剔除:新地標的 `已觀測/可觀測` 比例低於此值就刪除 | `0.3` | **N** |
| `Mapping.num_reliable_keyfrms` | 新地標要撐過幾個關鍵幀,才不會再被 `observed_ratio_thr` 這條規則檢查 | `2` | **N** |
| `Mapping.top_n_covisibilities_to_search` | 關鍵幀剔除掃描時檢查的前 N 個共視關鍵幀(`0` 完全停用關鍵幀剔除)——跟 `LoopDetector`/`Relocalizer` 同名參數是各自獨立的 | `30` | **N** |
| `Mapping.enable_interruption_of_landmark_generation` | 若有新關鍵幀進來,允許中途中斷新地標三角化 | `true` | **N** |
| `Mapping.enable_interruption_before_local_BA` | 若已經有排隊中的關鍵幀/暫停請求,直接跳過 local BA(關鍵幀原封不動排隊) | `true` | **N** |
| `Mapping.use_additional_keyframes_for_monocular` | 單目地圖若 local BA 裡固定的關鍵幀少於 2 個,強制多固定 2 個 local 關鍵幀以求穩定 | `false` | **N** |

*(寫死、沒開放 YAML 調整:local BA 固定跑 `num_first_iter=5` 然後排除離群值,再跑
`num_second_iter=10`——這兩個雖然是建構子參數,但 `local_bundle_adjuster_factory` 從來沒有從 YAML
餵值進去,目前沒辦法透過設定檔改。離群值卡方閾值跟追蹤那邊一樣是 5.99146/7.81473。
`two_view_triangulator` 的視差角閾值在 `mapping_module.cc:346` 呼叫處寫死 `1.0` 度,雖然這個類別本身
支援建構子參數。)*

---

## 5. 迴環偵測(Loop Closing)

原始碼:`module/loop_detector.{h,cc}`(區塊 `LoopDetector:`)

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `LoopDetector.enabled` | 啟動時迴環偵測器是否啟用 | `true` | Y |
| `LoopDetector.reject_by_graph_distance` | 用生成樹/迴環邊的圖距離排除候選,而非單純用共視關係 | `false` | Y |
| `LoopDetector.min_distance_on_graph` | 上述圖距離要多遠,關鍵幀才有資格成為迴環候選 | `50` | Y |
| `LoopDetector.backend` | Sim3 候選驗證用的姿態優化後端:`"g2o"`/`"gtsam"` | `"g2o"` | Y |
| `LoopDetector.num_final_matches_threshold` | Sim3 投影後,接受一個迴環所需的最少互相 2D-3D 配對數 | `40` | **N** |
| `LoopDetector.min_continuity` | 候選關鍵幀組合要連續被偵測到幾幀才驗證通過 | `3` | **N** |
| `LoopDetector.num_matches_thr` | 繼續測試一個原始候選所需的最少 BoW 配對數 | `20` | **N** |
| `LoopDetector.num_matches_thr_robust_matcher` | 額外做一次暴力配對後所需的最少配對數(`0` 停用這道額外程序) | `0`(關) | **N** |
| `LoopDetector.num_optimized_inliers_thr` | 非線性 Sim3 優化後,接受候選所需的最少內點數 | `20` | **N** |
| `LoopDetector.top_n_covisibilities_to_search` | 用前 N 個共視鄰居擴大候選/配對池(`0` 停用) | `0`(關) | **N** |
| `LoopDetector.use_fixed_seed` | 候選 PnP solver 用固定 RANSAC 種子 | `false` | **N** |
| `LoopDetector.num_common_words_thr_ratio` | 候選檢索用的 BoW 共同詞彙比例閾值 | `0.8` | **N** |
| `LoopDetector.g2o.num_trials_robust` / `.num_trials` / `.num_each_iter` | 意義同 `Tracking.g2o.*`,套用在 Sim3 候選姿態優化上 | `2`/`2`/`10` | **N** |
| `LoopDetector.gtsam.*`(6 個子參數) | 意義同 `Tracking.gtsam.*` | 同上 | **N** |

---

## 6. Global BA 與 Pose-Graph 優化(迴環閉合後)

原始碼:`global_optimization_module.{h,cc}`(**兩個獨立的同層級區塊,容易搞混**:`GlobalOptimizer:`
跟 `GraphOptimizer:`)、`optimize/graph_optimizer.{h,cc}`

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `GlobalOptimizer.num_iter` | 迴環閉合後,背景執行的全地圖 bundle adjustment 迭代數 | `10` | **N** |
| `GlobalOptimizer.use_huber_kernel` | 該次 global BA 是否使用 Huber 強健核函數 | `false` | **N** |
| `GlobalOptimizer.verbose` | 該次 global BA 的 g2o 詳細記錄 | `false` | **N** |
| `GlobalOptimizer.thr_neighbor_keyframes` | 迴環閉合當下,決定哪些共視關鍵幀要立即修正 Sim3 的最小共享地標數閾值 | `15` | P(只有 aist,設 100) |
| `GraphOptimizer.min_num_shared_lms` | Essential graph(Sim3 pose-graph 優化)裡,兩個關鍵幀要連邊所需的最少共享地標數 | `100` | P(只有 aist,設 200) |

*(寫死:Sim3 pose-graph 優化固定跑 `50` 次迭代,`graph_optimizer.cc:250`。)*

---

## 7. System

原始碼:`system.cc`(區塊 `System:`)

| YAML 路徑 | 說明 | 預設值 | 範例? |
|---|---|---|---|
| `System.map_format` | 地圖存/讀檔格式:`"msgpack"` 或 `"sqlite3"` | `"msgpack"` | Y |
| `System.min_num_shared_lms` | 兩個關鍵幀要在共視圖(covisibility graph)連邊所需的最少共同觀測地標數——是建圖 local BA 選鄰居、迴環偵測圖距離排除的基礎 | `15` | **N** |
| ~~`System.num_grid_cols`/`num_grid_rows`~~ | **這裡其實讀不到**——見文件最上面的 bug 說明,真正的 key 是 `Preprocessing.num_grid_cols`/`num_grid_rows` | — | — |

---

## 範圍之外,但程式碼裡有(順便記錄)

- `MarkerModel:` 區塊(`config.cc`):`type`(`"aruco"`/`"aruconano"`)、`width`、`marker_size`、
  `max_markers`、`dict`——ArUco AR 標記幾何設定,只有用標記做初始化/定尺度時才有作用,不算核心 SLAM
  調參。
- `StereoRectifier:` 區塊:相機校正用,不算演算法調參,見主 README。

## 完全寫死、無法透過 YAML 調整的關鍵常數(小抄)

這些是讀原始碼過程中發現、不看程式碼完全不會知道存在的東西,目前都沒辦法透過設定檔改,要改只能修
程式碼:

- 離群值排除:卡方閾值 `5.99146`(2 自由度/單目)、`7.81473`(3 自由度/雙目),對應 95% 信心水準——
  `pose_optimizer_g2o.cc` 跟 `local_bundle_adjuster_g2o.cc` 用的是同一組常數。
- Local BA 迭代次數 `num_first_iter=5`、`num_second_iter=10`——建構子參數存在,但
  `local_bundle_adjuster_factory` 從沒從 YAML 餵值。
- Pose-graph(Sim3)優化固定 `50` 次迭代。
- ORB FAST 搜尋格:`cell_size=64px`、`overlap=6px`、patch 半徑 `19px`。
- `frame_tracker` 的內點數門檻寫死 `10`(忽略 header 預設的 20)。
- `two_view_triangulator` 的視差角閾值在呼叫處寫死 `1.0`°。
- LiftFeat 擷取器:偵測閾值 `0.015`、最大特徵點數 `4096`。

## 原始碼快速導覽(依 pipeline 階段)

- 前端:`stella_vslam/src/stella_vslam/feature/orb_params.cc`、`feature/orb_extractor.cc`、`system.cc`
- 初始化:`stella_vslam/src/stella_vslam/module/initializer.cc`
- 追蹤:`stella_vslam/src/stella_vslam/tracking_module.cc`、`module/relocalizer.cc`、
  `module/keyframe_inserter.cc`、`module/frame_tracker.cc`、`optimize/pose_optimizer_factory.h`
- 建圖/local BA:`stella_vslam/src/stella_vslam/mapping_module.cc`、`module/local_map_cleaner.cc`、
  `optimize/local_bundle_adjuster_g2o.cc`、`optimize/local_bundle_adjuster_gtsam.cc`
- 迴環/global BA:`stella_vslam/src/stella_vslam/global_optimization_module.cc`、
  `module/loop_detector.cc`、`module/loop_bundle_adjuster.cc`、`optimize/graph_optimizer.cc`

## 怎麼有系統地做參數實驗

建議流程,搭配已經裝好的 `evo`(見 README「測試紀錄」章節)量化比較,不用只靠肉眼看軌跡像不像:

```bash
# 1. 複製一份 config,只改一個參數,方便之後對照是哪個參數造成的差異
cp stella_vslam/example/tum_rgbd/TUM_RGBD_mono_1.yaml configs/experiment_A.yaml
# 用編輯器改 configs/experiment_A.yaml 裡的某一個參數,例如 Feature.ini_fast_threshold: 20 → 10

# 2. 用改過的 config 重新跑,結果存到獨立資料夾
source env.sh
"${VSLAM_ROOT}/stella_vslam_examples/build/run_tum_rgbd_slam" \
  -v "$VOCAB_FILE" -d datasets/rgbd_dataset_freiburg1_xyz \
  -c configs/experiment_A.yaml \
  --no-sleep --auto-term --eval-log-dir datasets/eval_experiment_A --viewer none

# 3. 跟 baseline(沒改參數的那次)用 evo 比較數字
evo_ape tum datasets/rgbd_dataset_freiburg1_xyz/groundtruth.txt datasets/eval_experiment_A/frame_trajectory.txt -a
```

每次只改一個參數、留一份 baseline 當對照組,才看得出是哪個參數造成差異,而不是好幾個變因混在一起。
