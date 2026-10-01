#!/usr/bin/env python3
"""把 run_slam 的視覺化輸出跟真值錄到一個目錄,給 render_vslam_video.py 離線做影片(渲染不需要 ROS)。

run_slam 要開 publish_tracking_image / publish_frame_points / publish_map_points(isaac_vslam_ros_params.yaml
預設都開了,有人訂閱才發)。錄的內容(output_dir 底下,檔名是訊息時間戳 ns,模擬時間):

  camera/<ns>.jpg             ~/tracking_image/compressed:左影像 + 特徵點(綠 = 對到地圖點,橘 = 沒對到,暗區 = 遮罩)
  frame_points/<ns>.npy       ~/frame_points:當前影格特徵點用雙目深度轉成的 3D 點,N x 4(x y z tracked)
  map_points/<ns>.npy         ~/map_points:全部地圖點,N x 3(內容沒變就不重存)
  keyframes/<ns>.npy          ~/keyframes:關鍵幀相機位姿,M x 7(x y z qx qy qz qw,依關鍵幀 ID 排序)
  loop_edges/<ns>.npy         ~/loop_edges:迴環邊兩端的關鍵幀相機位置,K x 2 x 3(內容沒變就不重存)
  robot_pose.txt              ~/robot_pose(TUM,VSLAM 地圖座標系)
  camera_pose.txt             ~/camera_pose(TUM)
  gt_odom.txt                 真值 /odom(TUM,odom 座標系;Isaac Sim 的 /odom 是底盤真實位姿)
  tracking_state.txt          "<模擬時間> <Initializing|Tracking|Lost>"
  tf.txt                      "<時間> <parent> <child> x y z qx qy qz qw":靜態 TF <任何> -> vslam_map,
                              還有 map -> odom(AMCL,每秒最多一筆),用來把真值、佔據地圖換到 VSLAM 地圖座標系
  occupancy.npz               /map(機器人的 2D 佔據地圖,畫底圖用)

用法(VSLAM 啟動前或後都可以,Ctrl+C 結束):
  ros2 run vslam_bringup vslam_viz_recorder.py --ros-args -p output_dir:=/tmp/vslam_viz -p use_sim_time:=true
"""
import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def cloud_to_array(msg, names):
    """PointCloud2 裡指定的 FLOAT32 欄位 -> N x len(names) 的 float32 陣列。"""
    fields = {f.name: f for f in msg.fields}
    for name in names:
        if name not in fields or fields[name].datatype != PointField.FLOAT32:
            raise ValueError(f'PointCloud2 沒有 FLOAT32 欄位 {name}')
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, np.uint8).reshape(n, msg.point_step)
    cols = [raw[:, fields[name].offset:fields[name].offset + 4].copy().view(np.float32).reshape(n) for name in names]
    return np.stack(cols, axis=1) if cols else np.zeros((n, 0), np.float32)


def pose_line(t, p, q):
    return f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n'


class VizRecorder(Node):
    def __init__(self):
        super().__init__('vslam_viz_recorder')
        self.output_dir = self.declare_parameter('output_dir', '/tmp/vslam_viz').value
        prefix = self.declare_parameter('run_slam_prefix', '/run_slam').value.rstrip('/')
        self.map_frame = self.declare_parameter('map_frame', 'vslam_map').value
        self.occupancy_frame = self.declare_parameter('occupancy_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        # 預設收 run_slam 已經壓好的 JPEG(~/tracking_image/compressed):原始影像一張 2.7 MB,走 DDS 會掉幀
        self.compressed = bool(self.declare_parameter('compressed', True).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 90).value)
        # ~/keyframes 每幀都發,錄到這個間隔(秒,模擬時間)就夠畫軌跡了
        self.keyframes_interval = float(self.declare_parameter('keyframes_interval', 0.1).value)

        for sub in ('camera', 'frame_points', 'map_points', 'keyframes', 'loop_edges'):
            os.makedirs(os.path.join(self.output_dir, sub), exist_ok=True)
        self.files = {name: open(os.path.join(self.output_dir, name), 'a') for name in
                      ('robot_pose.txt', 'camera_pose.txt', 'gt_odom.txt', 'tracking_state.txt', 'tf.txt')}
        self.counts = {}
        self.last_map_points = None
        self.last_loop_edges = None
        self.last_keyframes_ns = -1
        self.last_map_to_odom_ns = -1
        self.static_saved = set()

        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self.compressed:
            self.create_subscription(CompressedImage, f'{prefix}/tracking_image/compressed', self.on_compressed,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        else:
            self.create_subscription(Image, f'{prefix}/tracking_image', self.on_image, reliable)
        self.create_subscription(PointCloud2, f'{prefix}/frame_points', self.on_frame_points, reliable)
        self.create_subscription(PointCloud2, f'{prefix}/map_points', self.on_map_points, reliable)
        self.create_subscription(PoseArray, f'{prefix}/keyframes', self.on_keyframes, reliable)
        self.create_subscription(Marker, f'{prefix}/loop_edges', self.on_loop_edges, reliable)
        self.create_subscription(Odometry, f'{prefix}/robot_pose', lambda m: self.on_pose(m, 'robot_pose.txt'), 50)
        self.create_subscription(Odometry, f'{prefix}/camera_pose', lambda m: self.on_pose(m, 'camera_pose.txt'), 50)
        self.create_subscription(String, f'{prefix}/tracking_state', self.on_state, latched)
        self.create_subscription(Odometry, self.declare_parameter('gt_odom_topic', '/odom').value,
                                 lambda m: self.on_pose(m, 'gt_odom.txt'), 50)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf_static, latched)
        self.create_subscription(TFMessage, '/tf', self.on_tf, QoSProfile(depth=100))
        self.create_subscription(OccupancyGrid, self.declare_parameter('occupancy_topic', '/map').value,
                                 self.on_occupancy, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                               durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'錄到 {self.output_dir}')

    def count(self, name):
        self.counts[name] = self.counts.get(name, 0) + 1

    def save_npy(self, sub, ns, array):
        np.save(os.path.join(self.output_dir, sub, f'{ns:019d}.npy'), array)
        self.count(sub)

    def on_compressed(self, msg):
        if 'jpeg' not in msg.format:
            self.get_logger().warn(f'不是 JPEG:{msg.format}', throttle_duration_sec=10.0)
            return
        with open(os.path.join(self.output_dir, 'camera', f'{stamp_ns(msg.header.stamp):019d}.jpg'), 'wb') as f:
            f.write(msg.data.tobytes() if hasattr(msg.data, 'tobytes') else bytes(msg.data))
        self.count('camera')

    def on_image(self, msg):
        channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
        if channels is None:
            self.get_logger().warn(f'不支援的影像編碼 {msg.encoding}', throttle_duration_sec=10.0)
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
        img = img.reshape(msg.height, msg.width, channels)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if ok:
            with open(os.path.join(self.output_dir, 'camera', f'{stamp_ns(msg.header.stamp):019d}.jpg'), 'wb') as f:
                f.write(jpg.tobytes())
            self.count('camera')

    def on_frame_points(self, msg):
        self.save_npy('frame_points', stamp_ns(msg.header.stamp), cloud_to_array(msg, ('x', 'y', 'z', 'tracked')))

    def on_map_points(self, msg):
        points = cloud_to_array(msg, ('x', 'y', 'z'))
        # 純定位時地圖不會變,同樣內容不重存
        if self.last_map_points is not None and np.array_equal(points, self.last_map_points):
            return
        self.last_map_points = points
        self.save_npy('map_points', stamp_ns(msg.header.stamp), points)

    def on_keyframes(self, msg):
        ns = stamp_ns(msg.header.stamp)
        if ns - self.last_keyframes_ns < self.keyframes_interval * 1e9:
            return
        self.last_keyframes_ns = ns
        poses = np.array([[p.position.x, p.position.y, p.position.z,
                           p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
                          for p in msg.poses], np.float64).reshape(-1, 7)
        self.save_npy('keyframes', ns, poses)

    def on_loop_edges(self, msg):
        points = np.array([[p.x, p.y, p.z] for p in msg.points], np.float64).reshape(-1, 2, 3)
        if self.last_loop_edges is not None and np.array_equal(points, self.last_loop_edges):
            return
        self.last_loop_edges = points
        self.save_npy('loop_edges', stamp_ns(msg.header.stamp), points)

    def on_pose(self, msg, name):
        t = stamp_ns(msg.header.stamp) * 1e-9
        self.files[name].write(pose_line(t, msg.pose.pose.position, msg.pose.pose.orientation))
        self.count(name)

    def on_state(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.files['tracking_state.txt'].write(f'{t:.9f} {msg.data}\n')
        self.files['tracking_state.txt'].flush()
        self.get_logger().info(f'tracking state: {msg.data}')

    def write_tf(self, tf):
        t = stamp_ns(tf.header.stamp) * 1e-9
        p, q = tf.transform.translation, tf.transform.rotation
        self.files['tf.txt'].write(f'{t:.9f} {tf.header.frame_id} {tf.child_frame_id} '
                                   f'{p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')
        self.files['tf.txt'].flush()

    def on_tf_static(self, msg):
        for tf in msg.transforms:
            key = (tf.header.frame_id, tf.child_frame_id, stamp_ns(tf.header.stamp))
            if tf.child_frame_id == self.map_frame and key not in self.static_saved:
                self.static_saved.add(key)
                self.write_tf(tf)
                self.get_logger().info(f'靜態 TF {tf.header.frame_id} -> {tf.child_frame_id}')

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == self.occupancy_frame and tf.child_frame_id == self.odom_frame:
                ns = stamp_ns(tf.header.stamp)
                if ns - self.last_map_to_odom_ns >= 1_000_000_000:
                    self.last_map_to_odom_ns = ns
                    self.write_tf(tf)

    def on_occupancy(self, msg):
        info = msg.info
        q = info.origin.orientation
        np.savez(os.path.join(self.output_dir, 'occupancy.npz'),
                 data=np.array(msg.data, np.int8).reshape(info.height, info.width),
                 resolution=info.resolution, frame_id=msg.header.frame_id,
                 origin=np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z,
                                  q.x, q.y, q.z, q.w]))
        self.get_logger().info(f'佔據地圖 {info.width}x{info.height} @ {info.resolution} m')

    def report(self):
        if self.counts:
            self.get_logger().info('已錄: ' + ', '.join(f'{k} {v}' for k, v in sorted(self.counts.items())))
        for f in self.files.values():
            f.flush()

    def close(self):
        for f in self.files.values():
            f.close()


def main():
    rclpy.init()
    node = VizRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.report()
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
