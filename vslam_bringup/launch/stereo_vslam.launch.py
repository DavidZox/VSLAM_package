"""雙目(ZED)VSLAM 部署 launch 檔。

啟動 stella_vslam_ros 套件的 run_slam 節點,吃兩支已經 rectify(校正對齊)過的左右影像 topic。
這支 launch 檔本身不做任何 SLAM 運算,只負責組裝參數、把 camera/left/right topic 接到實際相機
發布的 topic 上——運算全部由 stella_vslam_ros 提供。

前提:
  - stella_vslam_ros 已經在某個 colcon workspace 裡 build 好(見主 README「未來部署方式:ROS2」)
  - vocab 檔案(orb_vocab.fbow)——不進版控(檔案 43MB,不適合放進 git,見 .gitignore),但
    colcon_build.sh 會在 build 前自動準備好、跟著這個套件一起裝進 share/,所以預設值在任何裝過這個
    套件的機器上都能直接用,不用手動傳路徑;只有繞過 colcon_build.sh 直接 colcon build 時才需要自己
    指定 vocab_file:=
  - 已經有一個 ZED 影像來源節點(例如 zed-ros2-wrapper)在跑,發布已校正的左右影像 topic

用法:
  ros2 launch vslam_bringup stereo_vslam.launch.py \\
    left_image_topic:=/zed/zed_node/left/image_rect_color \\
    right_image_topic:=/zed/zed_node/right/image_rect_color
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('vslam_bringup')

    vocab_file_arg = DeclareLaunchArgument(
        'vocab_file',
        default_value=PathJoinSubstitution([pkg_share, 'vocab', 'orb_vocab.fbow']),
        description='ORB 詞彙檔路徑——colcon_build.sh 會自動把它裝進這個套件的 share 目錄,'
                     '不依賴任何特定機器的絕對路徑,不同機器上部署也能直接用'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'zed_stereo_vslam.yaml']),
        description='stella_vslam 相機/演算法設定 YAML —— 部署前一定要把裡面的 fx/fy/cx/cy/'
                     'focal_x_baseline 換成你那台 ZED 的實際校正值,見同目錄檔頭註解跟'
                     ' ../../docs/zed_stereo.md'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'vslam_ros_params.yaml']),
        description='stella_vslam_ros 節點的 ROS2 參數檔(TF frame id、是否發布 TF 等)'
    )
    viewer_arg = DeclareLaunchArgument(
        'viewer',
        default_value='none',
        description='none / pangolin_viewer / iridescence_viewer / socket_publisher。'
                     '機上無頭部署預設 none,要現場除錯看畫面才改別的(要另外裝、另外連得到畫面)'
    )
    left_topic_arg = DeclareLaunchArgument(
        'left_image_topic',
        default_value='/zed/zed_node/left/image_rect_color',
        description='ZED 左眼已校正(rectified)影像 topic。這是 zed-ros2-wrapper 一般常見的預設'
                     '命名慣例,但沒有在這台機器實際裝過 zed-ros2-wrapper 驗證過——實際名稱依'
                     'wrapper 版本、相機型號、啟動時的 namespace 參數而定,部署前務必自己'
                     '`ros2 topic list` 核對一次,不要照抄這個預設值'
    )
    right_topic_arg = DeclareLaunchArgument(
        'right_image_topic',
        default_value='/zed/zed_node/right/image_rect_color',
        description='ZED 右眼已校正影像 topic,同上,務必自己核對實際名稱'
    )

    run_slam_node = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='run_slam',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        arguments=[
            '-v', LaunchConfiguration('vocab_file'),
            '-c', LaunchConfiguration('config_file'),
            '--viewer', LaunchConfiguration('viewer'),
        ],
        # stella_vslam_ros 內部寫死訂閱 camera/left/image_raw、camera/right/image_raw,
        # 用 remapping 接到實際相機 topic,不用改它的原始碼
        remappings=[
            ('camera/left/image_raw', LaunchConfiguration('left_image_topic')),
            ('camera/right/image_raw', LaunchConfiguration('right_image_topic')),
        ],
    )

    return LaunchDescription([
        vocab_file_arg,
        config_file_arg,
        params_file_arg,
        viewer_arg,
        left_topic_arg,
        right_topic_arg,
        run_slam_node,
    ])
