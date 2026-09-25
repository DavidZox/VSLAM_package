#!/usr/bin/env bash
# 一個指令,用 colcon 把整條 ROS2 部署用的 stack 一起編譯:g2o -> stella_vslam -> stella_vslam_ros
# -> vslam_bringup(順序由 src/*/package.xml 的 <depend> 決定,colcon 自動排序,不用手動管順序)。
#
# 跟 build.sh 的差別/為什麼兩支腳本都留著:
#   build.sh(舊的,還留著)  -> 裝到 local_install/,給 stella_vslam_examples 用(本地端核心算法
#                              測試,run_experiment.sh 那一整套,不需要 ROS2,continue 照舊使用)
#   colcon_build.sh(這支)  -> 裝到 install/(colcon 慣例位置),給 stella_vslam_ros + vslam_bringup
#                              用(ROS2 部署)。
# 兩條路徑會**各自獨立重新編譯一次 g2o/stella_vslam**(build 產物分別在 g2o/build、stella_vslam/build
# vs colcon 自己的 build/g2o、build/stella_vslam),編譯時間跟磁碟空間會是兩份——這是刻意的取捨:
# 因為要讓 colcon build 真的「一個指令編完 ROS2 這邊所有東西」(包含 g2o/stella_vslam 本身,不是
# 當成外部已裝好的東西直接借用 local_install/),同時不影響 build.sh 那條已經驗證過、給本地端測試用
# 的路徑,兩者互不干擾。之後如果想省掉重複編譯,可以再討論怎麼讓 colcon 直接借用 local_install/,
# 但那樣 g2o/stella_vslam 就不是「colcon build 一起編出來的」了,跟目前的需求(全部一起編)矛盾。
#
# 用法:
#   source /opt/ros/<distro>/setup.bash   # 你的容器裡先 source 好 ROS2 環境
#   git submodule update --init --recursive
#   ./colcon_build.sh
#   source install/setup.bash
#   ros2 launch vslam_bringup stereo_vslam.launch.py ...

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if ! command -v colcon >/dev/null 2>&1; then
    echo "找不到 colcon,請先在容器裡裝好 ROS2 + colcon(這台開發機沒有 ROS2,這支腳本沒辦法在這裡跑)" >&2
    exit 1
fi

# 把 ORB 詞彙檔準備進 vslam_bringup/vocab/,讓 colcon 把它一起裝進 share/(見 vslam_bringup/CMakeLists.txt),
# 這樣 launch 檔的 vocab_file 預設路徑才不會綁死這台機器的絕對路徑,換一台機器部署也能直接用。
# 不進版控(檔案 43MB,見 .gitignore),優先複製本地端測試已經下載好的那份,沒有才現抓一份。
VOCAB_DST="$ROOT/vslam_bringup/vocab/orb_vocab.fbow"
if [ ! -f "$VOCAB_DST" ]; then
    mkdir -p "$ROOT/vslam_bringup/vocab"
    if [ -f "$ROOT/vocab/orb_vocab.fbow" ]; then
        cp "$ROOT/vocab/orb_vocab.fbow" "$VOCAB_DST"
    else
        curl -fL -o "$VOCAB_DST" https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow
    fi
fi

# g2o 跟 stella_vslam 都是純 CMake 專案(不是 ament 套件),各自需要不同的 -D 旗標才能正確配置。
# colcon 的 --cmake-args 是整個 build 一次套用給所有 cmake 類型套件的全域旗標,不是每個套件分開給——
# 這裡把兩邊需要的旗標直接合併成一份列表,對「不認得的那個套件」來說,多出來的 -D 變數 CMake 會
# 直接忽略(不會報錯、也不會動到該套件的行為),所以合併是安全的:
#   g2o 需要的(見 src/g2o/CMakeLists.txt):BUILD_SHARED_LIBS / G2O_USE_* / G2O_BUILD_*
#   stella_vslam 需要的:
#     - CMAKE_POLICY_VERSION_MINIMUM=3.5:它自己跟它的巢狀 submodule 3rd/tinycolormap 都宣告
#       cmake_minimum_required(VERSION 3.1),CMake 4.x 直接拒絕,要用這個旗標繞過去
#     - BOW_FRAMEWORK=FBoW / USE_STACK_TRACE_LOGGER=OFF:跟本地端 build.sh 用的設定一致
colcon build --base-paths src \
    --cmake-args \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DBUILD_SHARED_LIBS=ON \
        -DG2O_USE_CHOLMOD=OFF \
        -DG2O_USE_CSPARSE=ON \
        -DG2O_USE_OPENGL=OFF \
        -DG2O_USE_OPENMP=OFF \
        -DG2O_BUILD_EXAMPLES=OFF \
        -DG2O_BUILD_APPS=OFF \
        -DBOW_FRAMEWORK=FBoW \
        -DUSE_STACK_TRACE_LOGGER=OFF

echo
echo "==> Done. 用法:"
echo "    source $ROOT/install/setup.bash"
echo "    ros2 launch vslam_bringup stereo_vslam.launch.py"
