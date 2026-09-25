# src/ 是什麼

這個資料夾裡沒有任何「真正的」檔案——四個都是 symlink(捷徑/別名),指回 `VSLAM_package` 根目錄底下
同名的資料夾:

```
src/g2o               -> ../g2o
src/stella_vslam       -> ../stella_vslam
src/stella_vslam_ros   -> ../stella_vslam_ros
src/vslam_bringup      -> ../vslam_bringup
```

你在 `src/g2o/` 裡看到一堆檔案,是因為 symlink 會「透明地」顯示它指向的那個資料夾的真實內容——
`src/g2o/` 跟 `../g2o/`(也就是 `VSLAM_package/g2o/`)**是同一份東西,不是兩份**。用 `src/g2o/xxx.cpp`
這條路徑打開的檔案,跟用 `g2o/xxx.cpp` 打開的,是完全同一個檔案,改其中一邊,另一邊也會跟著變,因為
它們本來就是同一個檔案,只是有兩條不同的路徑可以走到它。

## symlink 是怎麼設定的

**沒有設定檔**,建立 symlink 純粹是下一個指令,不需要先編輯任何檔案:

```bash
ln -s ../g2o src/g2o
```

執行這一行的當下,系統就直接在 `src/` 資料夾裡建立了一個新的 symlink 項目(名字叫 `g2o`,指向
`../g2o`),指令跑完這件事就完成了,不會有「先寫設定檔、再套用設定」這兩個步驟。

跟你比較熟悉的 git submodule 比較一下,會更清楚差在哪:

| 情境 | 怎麼設定 |
|---|---|
| git submodule | 先編輯 `.gitmodules`(一份文字檔),再跑 `git submodule update` 讓它生效——分兩步,**有設定檔** |
| symlink | 直接跑 `ln -s 目標 名稱` 這一個指令——一步就完成,**沒有設定檔** |

`src/` 底下這四個 symlink 就是這樣建立的:

```bash
ln -s ../g2o src/g2o
ln -s ../stella_vslam src/stella_vslam
ln -s ../stella_vslam_ros src/stella_vslam_ros
ln -s ../vslam_bringup src/vslam_bringup
```

之後要建、改、刪,也都是直接下指令,不會去改某個檔案:

```bash
ln -s ../g2o src/g2o        # 建立
rm src/g2o                   # 刪除(只刪捷徑本身,../g2o 完全不受影響)
ln -sf ../新資料夾 src/g2o   # 改指向(-f 是覆蓋既有的)
```

想知道某個 symlink 目前指去哪,可以用 `readlink`(直接讀出它自己存的目標路徑,不用去查任何設定檔):

```bash
readlink src/g2o
# ../g2o
```

## 所以我的原始碼到底在哪裡、要在哪裡改

**只有一份,在 `VSLAM_package` 根目錄底下**(`g2o/`、`stella_vslam/`、`stella_vslam_ros/`、
`vslam_bringup/`),`src/` 底下沒有獨立的第二份。之前說「這些是內部機制,只有要改演算法才需要進去」,
應該拆成兩件事講清楚,不然容易混淆:

- **`src/` 這個資料夾本身**,才是純粹的機制——它唯一的用途是讓 `colcon build` 找得到套件在哪
  (`colcon` 慣例上會去專案的 `src/` 底下找),平常**不需要透過 `src/` 去改檔案**,直接到根目錄的
  `g2o/`、`stella_vslam/` 等資料夾改就好,結果完全一樣,只是路徑比較短、比較直覺。
- **`g2o/`、`stella_vslam/`、`stella_vslam_ros/` 這三個資料夾本身,是貨真價實的原始碼**,只是不是
  你自己寫的(是用 git submodule 引用的別人開源專案,細節見主 `README.md`「版控說明」)——所以「只有
  要改演算法才需要進去」指的是這三個,不是說它們是空的或不算原始碼。

## 各資料夾實際放什麼

| 資料夾 | 內容 | 誰維護 | 你什麼時候會進去改 |
|---|---|---|---|
| `g2o/` | 圖優化(graph optimization)函式庫,`stella_vslam` 內部算 bundle adjustment 用的數學後端 | 上游 `RainerKuemmerle/g2o`,fork 成 `DavidZox/g2o` | 幾乎不會,除非要動優化算法本身或修 build 設定(例如之前修過的 CMake config bug) |
| `stella_vslam/` | 核心 VSLAM 演算法本體:特徵擷取、追蹤、建圖、迴環偵測都在這裡 | 上游 `stella-cv/stella_vslam`,fork 成 `DavidZox/stella_vslam` | 之後你說要自己修改開發演算法,會進來動的就是這裡 |
| `stella_vslam_ros/` | 把 `stella_vslam` 包成 ROS2 節點:訂閱相機 topic、發布位姿/TF | 上游 `stella-cv/stella_vslam_ros`,fork 成 `DavidZox/stella_vslam_ros` | 要改 ROS2 節點本身的行為(不是參數,是程式邏輯)才會進來,例如加新的 topic、改發布邏輯 |
| `vslam_bringup/` | 你自己的部署設定:launch 檔 + YAML 參數,不含任何演算法邏輯 | 你自己(不是 submodule,直接在 `VSLAM_package` 自己的版控裡) | 這是你平常最常會碰的——調 ZED 校正參數、TF frame 名稱、topic remap 等,都在這裡改 |

## 改了 g2o/stella_vslam/stella_vslam_ros 裡面的東西之後呢

因為這三個是 submodule(獨立的 git repo),改完不能直接在 `VSLAM_package` 這層 commit 就結束,要照
主 `README.md`「版控說明」裡「兩層都 push」的流程:先進 `g2o/`(或 `stella_vslam/`、
`stella_vslam_ros/`)自己的資料夾 commit + push 到對應的 fork,確定推上去之後,再回 `VSLAM_package`
根目錄把 submodule 指標更新的那個 commit 推上去。這件事不管你是透過 `src/g2o/` 還是直接
`VSLAM_package/g2o/` 去改都一樣——因為是同一份檔案,git 也只認 `g2o/` 這個 submodule 本身,不會管你是
從哪條路徑進去改的。
