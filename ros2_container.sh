#!/usr/bin/env bash
# 在 Docker 容器裡編譯/執行 VSLAM 的 ROS2 部分(給沒有裝 ROS2 的主機用,例如接 Isaac Sim 的 DGX Spark)。
#
# 容器直接用機器人端的映像 amr_base(裡面已經有 ROS2 Jazzy、cv_bridge、image_transport、OpenCV、Eigen、
# SuiteSparse 等 colcon_build.sh 需要的東西),把這個 repo 掛到 /workspaces/VSLAM_package,網路/IPC 跟主機共用,
# 所以跟 Isaac Sim、amr-base-dev 容器在同一個 DDS domain 互通。編譯產物(build/ install/ log/)留在 repo 裡。
#
# 用法:
#   ./ros2_container.sh up                建立並啟動容器(已存在就直接啟動)
#   ./ros2_container.sh build             在容器裡跑 ./colcon_build.sh
#   ./ros2_container.sh launch [args...]  ros2 launch vslam_bringup isaac_stereo_vslam.launch.py [args...]
#   ./ros2_container.sh run <cmd...>      在容器裡(已 source ROS2 + install/)執行任意指令
#   ./ros2_container.sh shell             進容器的互動式 shell
#   ./ros2_container.sh stop              送 SIGINT 給容器裡的 VSLAM(run_slam 收到才會存 map_db_out / eval_log_dir)
#   ./ros2_container.sh down              刪除容器(repo 裡的編譯產物不受影響)
#
# 環境變數:
#   VSLAM_CONTAINER  容器名稱,預設 vslam-dev
#   VSLAM_IMAGE      映像,預設 amr_base:latest(在 fih_humanoid_amr/amr_base 用 docker compose build 產生)
#   ROS_DOMAIN_ID    直接指定 domain;沒給就讀 $AMR_BASE_DIR/domain_id.conf 的 TARGET_DOMAIN_ID
#   AMR_BASE_DIR     機器人 repo 的 amr_base 目錄,預設 ~/Gitlab_workspace/fih_humanoid_amr/amr_base
#
# 注意:機器人端 launch 關閉時會 pkill -9 -f "ros2"(amr-base-dev 是 pid: host,看得到主機上所有行程),
# 這邊的 ros2 launch / ros2 run 父行程也會被砍掉(底下的 run_slam 等子行程可能變成孤兒繼續跑),
# 重啟機器人端之後,這邊建議也整個重啟(./ros2_container.sh stop 再重新 launch)。

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${VSLAM_CONTAINER:-vslam-dev}"
IMAGE="${VSLAM_IMAGE:-amr_base:latest}"
AMR_BASE_DIR="${AMR_BASE_DIR:-$HOME/Gitlab_workspace/fih_humanoid_amr/amr_base}"
WS=/workspaces/VSLAM_package

domain_id() {
    if [ -n "${ROS_DOMAIN_ID:-}" ]; then
        echo "$ROS_DOMAIN_ID"
    elif [ -f "$AMR_BASE_DIR/domain_id.conf" ]; then
        (source "$AMR_BASE_DIR/domain_id.conf" && echo "${TARGET_DOMAIN_ID:?domain_id.conf 沒有 TARGET_DOMAIN_ID}")
    else
        echo "找不到 $AMR_BASE_DIR/domain_id.conf,請用 ROS_DOMAIN_ID=<n> 指定" >&2
        return 1
    fi
}

container_exists() { docker container inspect "$CONTAINER" >/dev/null 2>&1; }
container_running() { [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]; }

up() {
    if container_exists; then
        container_running || docker start "$CONTAINER" >/dev/null
        echo "容器 $CONTAINER 已啟動"
        return
    fi
    local gpu_args=()
    if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
        gpu_args=(--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all)
    fi
    docker run -d --name "$CONTAINER" --hostname "$CONTAINER" \
        --network host --ipc host "${gpu_args[@]}" \
        -e DISPLAY="${DISPLAY:-:0}" -e TZ=Asia/Taipei \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="file://$WS/vslam_bringup/config/cyclonedds.xml" \
        -v "$ROOT:$WS" -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro \
        -w "$WS" "$IMAGE" sleep infinity >/dev/null
    echo "已建立容器 $CONTAINER(映像 $IMAGE,repo 掛在 $WS)"
}

# 在容器裡執行 bash 指令字串,先 source ROS2 跟 install/(有的話)。
# 第二個參數給 noinstall 就不 source install/:編譯時不能 source,否則 stella_vslam 的 CMake 會
# find_package 到 install/ 裡上一次裝的 FBoW,連帶用到 install/stella_vslam/include 的舊標頭。
in_container() {
    local tty_args=(-i)
    [ -t 0 ] && [ -t 1 ] && tty_args=(-it)
    local domain setup='{ [ ! -f install/setup.bash ] || source install/setup.bash; } &&'
    [ "${2:-}" = noinstall ] && setup=''
    domain="$(domain_id)"
    container_running || up >/dev/null
    docker exec "${tty_args[@]}" -e ROS_DOMAIN_ID="$domain" -w "$WS" "$CONTAINER" bash -c \
        "source /opt/ros/jazzy/setup.bash && $setup $1"
}

cmd="${1:-}"
[ $# -gt 0 ] && shift
case "$cmd" in
    up) up ;;
    build) in_container "./colcon_build.sh" noinstall ;;
    launch) in_container "exec ros2 launch vslam_bringup isaac_stereo_vslam.launch.py $(printf '%q ' "$@")" ;;
    run) [ $# -gt 0 ] || { echo "用法: $0 run <cmd...>" >&2; exit 1; }; in_container "$(printf '%q ' "$@")" ;;
    shell) in_container "echo \"ROS_DOMAIN_ID=\$ROS_DOMAIN_ID\"; exec bash" ;;
    stop)
        # [r]/[s] 讓 pattern 不會比對到 pkill 自己所在的 bash -c 指令列
        docker exec "$CONTAINER" bash -c 'pkill -INT -f "[r]os2 launch vslam_bringup"; pkill -INT -f "[s]tella_vslam_ros/run_slam"; \
            for _ in $(seq 30); do pgrep -f "[s]tella_vslam_ros/run_slam" >/dev/null || exit 0; sleep 1; done; \
            echo "run_slam 30 秒內沒有結束(可能還在存地圖)" >&2; exit 1' && echo "VSLAM 已停止" ;;
    down) docker rm -f "$CONTAINER" >/dev/null 2>&1 && echo "已刪除容器 $CONTAINER" || echo "沒有容器 $CONTAINER" ;;
    *) sed -n '2,26p' "$0"; exit 1 ;;
esac
