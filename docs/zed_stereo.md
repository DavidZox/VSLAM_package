# ZED 雙目相機 — stereo 設定與接入筆記

未來的實際應用預計用 ZED(Stereolabs)雙目相機。這份筆記記錄怎麼拿到 ZED 的校正參數、怎麼填進
stella_vslam 的 stereo 設定,以及一個目前還沒解決、需要先知道的技術限制。目前手上沒有實體 ZED,
所以下面數值都是**佔位符**,拿到相機後才能填真實數字、實際測試。

## 1. ZED 的雙目校正原理(跟一般 DIY 雙目相機不一樣)

ZED 系列是出廠就校正好的雙目相機,序號跟校正參數綁定,而且 **ZED SDK 預設輸出的左右影像已經是
rectify(對齊校正)過的**——這點很重要,代表接進 stella_vslam 時可以直接用最簡單的 stereo 設定格式
(只填 `fx/fy/cx/cy` + `focal_x_baseline`),不需要 `stella_vslam/example/euroc/EuRoC_stereo.yaml`
裡那種 `StereoRectifier`(原始 K/D/R 矩陣,by stella_vslam 自己在跑的時候即時 rectify)區塊。
`configs/ZED_stereo.yaml` 就是照這個前提寫的範本。

## 2. 怎麼拿到真實校正數值(拿到相機後,擇一)

1. **ZED SDK API**(裝了 ZED SDK 之後,程式裡呼叫):
   `sl::Camera::getCameraInformation().camera_configuration.calibration_parameters`,
   裡面有 `left_cam`/`right_cam` 的 fx/fy/cx/cy/畸變係數,以及雙目的 baseline。
2. **官方校正檔**(不用裝 SDK,只要知道相機序號):瀏覽器打開
   `https://calib.stereolabs.com/?SN=<你的相機序號>`,會下載一份 `.conf` 檔(INI 格式),依解析度
   分段,例如 `[LEFT_CAM_HD]`/`[RIGHT_CAM_HD]`/`[STEREO]`,裡面就有 fx/fy/cx/cy/k1/k2/p1/p2/k3,
   還有 `[STEREO]` 段的 `Baseline`(單位毫米)。

## 3. 填入 `configs/ZED_stereo.yaml`

範本裡每個要換的欄位都標了 `# TODO`:

- `fx`/`fy`/`cx`/`cy`:對應解析度那個 section 直接抄
- `cols`/`rows`/`fps`:換成你實際擷取時用的解析度/幀率(ZED 有多種模式可選,HD720/HD1080/HD2K 等,
  數值要跟你選的模式對上)
- `focal_x_baseline`:`.conf` 檔 `[STEREO]` 段的 `Baseline` 除以 1000(毫米轉公尺)再乘上 `fx`

## 4. ⚠️ 目前發現的限制:`run_camera_slam` 的 stereo 模式接不了 ZED 這種單一 USB 裝置

檢查過 `stella_vslam_examples/src/run_camera_slam.cc` 的 `stereo_tracking()` 實作
(`src/run_camera_slam.cc:295-300` 附近):它是開**兩個獨立的 `cv::VideoCapture`**
(裝置編號 `cam_num` 跟 `cam_num + 1`),預期左右眼是作業系統看到的兩個不同 `/dev/videoX` 裝置。

但 ZED 對 USB/作業系統來說是**單一個裝置**,吐出的是左右影像左右拼接在一起的單一畫面(例如 720p
模式整體輸出 2560x720),不是兩個獨立裝置——所以 `run_camera_slam` 現在這個 stereo 模式**沒辦法
直接接 ZED**,需要多一層處理。

### 建議的路(配合未來 ROS2 部署,不用特別為本地端測試多寫工具):

因為之後本來就要走 `stella_vslam_ros` 部署(見主 `README.md`「未來部署方式」章節),Stereolabs 官方
自己提供 [`zed-ros2-wrapper`](https://github.com/stereolabs/zed-ros2-wrapper),會把 ZED 拆成
**已經 rectify 好的獨立 topic**發布出來(左右眼分開的 topic),格式正好對上 `stella_vslam_ros`
本來就期待的 `camera/left/image_raw`、`camera/right/image_raw` 雙目訂閱——這條路完全不用管
`run_camera_slam` 那個雙裝置限制,是接 ZED 最乾淨的方式,等 `stella_vslam_ros` 那邊開始建置時一併
處理即可。

如果想在那之前,先用本地端這幾支 example 執行檔測 stereo pipeline(不透過 ROS2),需要另外寫一支
小程式:用 ZED SDK 抓已經 rectify 好的左右 `sl::Mat`、轉成 `cv::Mat`,直接呼叫 stella_vslam 的
`system::feed_stereo_frame(left, right, timestamp, mask)` API(`run_camera_slam.cc:369` 那行
用的就是這個函式)——概念上不難,但要多寫這支橋接程式,目前先記錄下來,等真的需要本地端 stereo
測試再動手。

## 5. 待辦

- [ ] 拿到實體 ZED 之後,用上面兩種方式之一取得真實校正參數,填進 `configs/ZED_stereo.yaml`
- [ ] 決定要先寫本地端橋接程式測 stereo,還是直接等 `stella_vslam_ros` + `zed-ros2-wrapper` 一起弄
- [ ] 確認 ZED 型號(ZED / ZED 2 / ZED 2i / ZED Mini / ZED X 的 baseline、感光元件不同,影響上面的數值)
