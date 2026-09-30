"""Isaac Sim 機器人(fih_humanoid_amr 場景 + amr_base_sw)的雙目 VSLAM launch 檔。

跟 stereo_vslam.launch.py(實體 ZED)差在:
  - 影像來源是 Isaac Sim 的 /zed_camera_left|right/image_raw,參數用 config/isaac_*.yaml
  - 機器人 URDF 沒有相機 link,這裡依 config/isaac_zedx_camera_tf.yaml 補上 L_HEAD_J2 -> 相機的靜態 TF
  - 相機看得到機器人自己的雙手,預設帶 config/isaac_zedx_self_mask.png 遮罩(make_self_mask.py 產生)
  - 全部節點用模擬時間(use_sim_time)
  - /initialpose 預設改接 /vslam/initialpose,不會吃到機器人端送給 AMCL 的初始位姿

前提:Isaac Sim 場景已經在 play,機器人端 start_sim_and_spawn_IsaacSim.launch.py 已經在跑(提供 TF 跟 /clock),
ROS_DOMAIN_ID 跟機器人端一樣(amr_base/domain_id.conf)。

用法:
  ros2 launch vslam_bringup isaac_stereo_vslam.launch.py
  # 建圖並在結束(Ctrl+C)時存地圖、存軌跡:
  ros2 launch vslam_bringup isaac_stereo_vslam.launch.py map_db_out:=/tmp/isaac_map.msg eval_log_dir:=/tmp/vslam_eval
  # 載入地圖只做定位:
  ros2 launch vslam_bringup isaac_stereo_vslam.launch.py map_db_in:=/tmp/isaac_map.msg disable_mapping:=true
  # 讓 VSLAM 接手 map -> odom(機器人端不能同時跑 AMCL / slam_toolbox):
  ros2 launch vslam_bringup isaac_stereo_vslam.launch.py publish_tf:=true map_frame:=map
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_SHARE = get_package_share_directory('vslam_bringup')
# 相機 link(x 前 y 左 z 上)-> 光學座標系(x 右 y 下 z 前)的固定旋轉
OPTICAL_RPY = ('-1.5707963267948966', '0', '-1.5707963267948966')


def camera_tf_nodes(tf_file, use_sim_time):
    with open(tf_file) as f:
        cfg = yaml.safe_load(f)
    nodes = []
    for cam in cfg['cameras']:
        x, y, z = (str(v) for v in cam['xyz'])
        roll, pitch, yaw = (str(v) for v in cam['rpy'])
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f"{cam['link_frame']}_tf",
            arguments=['--x', x, '--y', y, '--z', z, '--roll', roll, '--pitch', pitch, '--yaw', yaw,
                       '--frame-id', cfg['parent_frame'], '--child-frame-id', cam['link_frame']],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f"{cam['optical_frame']}_tf",
            arguments=['--roll', OPTICAL_RPY[0], '--pitch', OPTICAL_RPY[1], '--yaw', OPTICAL_RPY[2],
                       '--frame-id', cam['link_frame'], '--child-frame-id', cam['optical_frame']],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    return nodes


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    actions = []
    use_sim_time = arg('use_sim_time').lower() == 'true'

    run_slam_args = ['-v', arg('vocab_file'), '-c', arg('config_file'), '--viewer', arg('viewer'),
                     '--log-level', arg('log_level')]
    mask = arg('mask')
    if mask and mask.lower() != 'none':
        if not os.path.isfile(mask):
            raise RuntimeError(f'找不到遮罩 {mask}(用 make_self_mask.py 產生,或 mask:=none 不用遮罩)')
        run_slam_args += ['--mask', mask]
    if arg('map_db_in'):
        if not os.path.isfile(arg('map_db_in')):
            raise RuntimeError(f"找不到地圖檔 {arg('map_db_in')}")
        run_slam_args += ['-i', arg('map_db_in')]
    if arg('map_db_out'):
        # 結束時才寫檔,目錄不存在會存失敗
        os.makedirs(os.path.dirname(os.path.abspath(arg('map_db_out'))), exist_ok=True)
        run_slam_args += ['-o', arg('map_db_out')]
    if arg('eval_log_dir'):
        # run_slam 要求目錄已存在
        os.makedirs(arg('eval_log_dir'), exist_ok=True)
        run_slam_args += ['--eval-log-dir', arg('eval_log_dir')]
    if arg('disable_mapping').lower() == 'true':
        run_slam_args += ['--disable-mapping']

    # 空字串 = 照 params_file,有給才覆寫
    overrides = {'use_sim_time': use_sim_time}
    if arg('publish_tf'):
        overrides['publish_tf'] = arg('publish_tf').lower() == 'true'
        if overrides['publish_tf']:
            # map -> odom 由 VSLAM 發的時候,不能再把 vslam 地圖掛在 odom 底下(會變成迴圈)
            overrides['map_parent_frame'] = ''
    if arg('map_frame'):
        overrides['map_frame'] = arg('map_frame')
    if arg('map_parent_frame'):
        overrides['map_parent_frame'] = '' if arg('map_parent_frame').lower() == 'none' else arg('map_parent_frame')

    if arg('publish_camera_tf').lower() == 'true':
        actions += camera_tf_nodes(arg('camera_tf_file'), use_sim_time)

    actions.append(Node(
        package='stella_vslam_ros', executable='run_slam', name='run_slam', output='screen',
        parameters=[arg('params_file'), overrides],
        arguments=run_slam_args,
        # run_slam 內部寫死訂閱 camera/left|right/image_raw 跟 /initialpose,用 remap 接到實際 topic
        remappings=[
            ('camera/left/image_raw', arg('left_image_topic')),
            ('camera/right/image_raw', arg('right_image_topic')),
            ('/initialpose', arg('initial_pose_topic')),
        ],
    ))
    if arg('rviz').lower() == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', name='vslam_rviz', output='log',
            arguments=['-d', os.path.join(PKG_SHARE, 'rviz', 'isaac_vslam.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    actions.insert(0, LogInfo(msg=f"run_slam {' '.join(run_slam_args)}"))
    return actions


def generate_launch_description():
    config_dir = os.path.join(PKG_SHARE, 'config')
    args = [
        ('left_image_topic', '/zed_camera_left/image_raw', 'Isaac Sim 左眼影像(Scene 的 HeadCamera 圖發布)'),
        ('right_image_topic', '/zed_camera_right/image_raw', 'Isaac Sim 右眼影像'),
        ('vocab_file', os.path.join(PKG_SHARE, 'vocab', 'orb_vocab.fbow'), 'ORB 詞彙檔'),
        ('config_file', os.path.join(config_dir, 'isaac_zedx_stereo.yaml'), 'stella_vslam 相機/演算法設定'),
        ('params_file', os.path.join(config_dir, 'isaac_vslam_ros_params.yaml'), 'run_slam 的 ROS2 參數'),
        ('mask', os.path.join(config_dir, 'isaac_zedx_self_mask.png'),
         '機器人本體遮罩 PNG(0 = 不抽特徵);none = 不用遮罩'),
        ('camera_tf_file', os.path.join(config_dir, 'isaac_zedx_camera_tf.yaml'), '相機靜態 TF 設定'),
        ('publish_camera_tf', 'true', '發布 L_HEAD_J2 -> 相機的靜態 TF;URDF 自己有相機 link 時改 false'),
        ('use_sim_time', 'true', '用 Isaac Sim 的 /clock'),
        ('publish_tf', '', '覆寫 params_file 的 publish_tf(true = VSLAM 發 map -> odom)'),
        ('map_frame', '', '覆寫 params_file 的 map_frame'),
        ('map_parent_frame', '', '覆寫 params_file 的 map_parent_frame(odom / map;none = 不發 parent -> vslam_map)'),
        ('initial_pose_topic', '/vslam/initialpose', '給 VSLAM 的初始位姿 topic(預設不接 AMCL 用的 /initialpose)'),
        ('map_db_in', '', '載入地圖檔(-i)'),
        ('map_db_out', '', '結束時存地圖檔(-o)'),
        ('eval_log_dir', '', '結束時存 frame/keyframe 軌跡(TUM 格式)跟追蹤耗時的目錄'),
        ('disable_mapping', 'false', 'true = 只定位不建圖(搭配 map_db_in)'),
        ('viewer', 'none', 'none / pangolin_viewer / iridescence_viewer / socket_publisher'),
        ('rviz', 'false', 'true = 另外開一個 RViz(rviz/isaac_vslam.rviz):真值 /odom vs VSLAM 軌跡、關鍵幀、追蹤影像'),
        ('log_level', 'info', 'stella_vslam 的 log 等級'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc) for name, default, desc in args]
        + [OpaqueFunction(function=launch_setup)])
