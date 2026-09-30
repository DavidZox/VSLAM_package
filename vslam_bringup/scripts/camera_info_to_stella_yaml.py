#!/usr/bin/env python3
"""從左右 camera_info 算出 stella_vslam 雙目設定的 Camera 區塊,或檢查現有設定檔跟 topic 對不對得上。

  - fx/fy/cx/cy/cols/rows/畸變:左眼 camera_info
  - 基線:右眼 P[3](= -fx * baseline)不是 0 就用它;Isaac Sim 各相機獨立發布,P[3] 是 0,這時改查 TF
    左光學座標系 -> 右光學座標系的距離(isaac_stereo_vslam.launch.py 有發這段靜態 TF),都沒有就用 baseline 參數
  - fps:camera_info 時間戳間隔的中位數(模擬時是模擬時間)
左右內參不同或有畸變時會警告:stella_vslam 雙目要吃已校正(rectified)的影像。

用法:
  ros2 run vslam_bringup camera_info_to_stella_yaml.py --ros-args -p use_sim_time:=true
      # 印出 Camera 區塊,貼進 stella_vslam 設定檔
  ros2 run vslam_bringup camera_info_to_stella_yaml.py --ros-args -p use_sim_time:=true \\
      -p check:=$(ros2 pkg prefix vslam_bringup)/share/vslam_bringup/config/isaac_zedx_stereo.yaml
      # 比對設定檔,不一致時 exit code 1
"""
import math
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener


class CameraInfoProbe(Node):
    def __init__(self):
        super().__init__('camera_info_to_stella_yaml')
        self.left_topic = self.declare_parameter('left_info_topic', '/zed_camera_left/camera_info').value
        self.right_topic = self.declare_parameter('right_info_topic', '/zed_camera_right/camera_info').value
        self.baseline_param = float(self.declare_parameter('baseline', 0.0).value)
        self.check = self.declare_parameter('check', '').value
        self.samples = int(self.declare_parameter('samples', 20).value)
        self.left, self.right, self.stamps = None, None, []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.left_topic, self.on_left, 10)
        self.create_subscription(CameraInfo, self.right_topic, self.on_right, 10)

    def on_left(self, msg):
        self.left = msg
        self.stamps.append(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)

    def on_right(self, msg):
        self.right = msg

    def baseline(self):
        fx = self.right.p[0]
        if abs(self.right.p[3]) > 1e-9 and fx > 0:
            return -self.right.p[3] / fx, f'{self.right_topic} 的 P[3]'
        end = time.monotonic() + 3.0
        while time.monotonic() < end and not self.tf_buffer.can_transform(
                self.left.header.frame_id, self.right.header.frame_id, Time()):
            rclpy.spin_once(self, timeout_sec=0.05)
        try:
            tf = self.tf_buffer.lookup_transform(self.left.header.frame_id, self.right.header.frame_id, Time())
            t = tf.transform.translation
            return math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2), \
                f'TF {self.left.header.frame_id} -> {self.right.header.frame_id}'
        except TransformException as ex:
            if self.baseline_param > 0:
                return self.baseline_param, 'baseline 參數'
            raise RuntimeError(f'P[3] 是 0、TF 也查不到({ex}),請用 -p baseline:=<公尺> 指定') from ex

    def compute(self):
        L, R = self.left, self.right
        warnings = []
        if list(L.k) != list(R.k) or (L.width, L.height) != (R.width, R.height):
            warnings.append('左右內參/解析度不同:影像可能沒校正,stella_vslam 雙目需要 rectified 影像(或設 StereoRectifier + --rectify)')
        if any(abs(v) > 1e-9 for v in L.d) or any(abs(v) > 1e-9 for v in R.d):
            warnings.append(f'有畸變係數 {list(L.d)}:雙目請先校正影像')
        baseline, source = self.baseline()
        dts = sorted(b - a for a, b in zip(self.stamps, self.stamps[1:]) if b > a)
        fps = 1.0 / dts[len(dts) // 2] if dts else 30.0
        cam = {
            'fx': L.k[0], 'fy': L.k[4], 'cx': L.k[2], 'cy': L.k[5],
            'k1': 0.0, 'k2': 0.0, 'p1': 0.0, 'p2': 0.0, 'k3': 0.0,
            'focal_x_baseline': L.k[0] * baseline,
            'fps': round(fps, 3), 'cols': L.width, 'rows': L.height,
        }
        return cam, baseline, source, warnings

    def check_config(self, cam):
        with open(self.check) as f:
            cfg = yaml.safe_load(f)['Camera']
        bad = []
        for key, value in cam.items():
            have = cfg.get(key)
            tol = 0.01 * abs(value) + 1e-6 if key in ('focal_x_baseline', 'fps') else 1e-3
            if have is None or abs(float(have) - float(value)) > tol:
                bad.append(f'  {key}: 設定檔 {have},topic 量到 {value}')
        return bad


def main():
    rclpy.init()
    node = CameraInfoProbe()
    start = time.monotonic()
    while rclpy.ok() and (node.left is None or node.right is None or len(node.stamps) < node.samples):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - start > 30.0:
            print(f'30 秒內收不齊 {node.left_topic} / {node.right_topic}', file=sys.stderr)
            return 2
    try:
        cam, baseline, source, warnings = node.compute()
    except RuntimeError as ex:
        print(ex, file=sys.stderr)
        return 2
    for w in warnings:
        print(f'# 警告: {w}', file=sys.stderr)
    code = 0
    if node.check:
        bad = node.check_config(cam)
        if bad:
            print(f'{node.check} 跟 topic 不一致:\n' + '\n'.join(bad))
            code = 1
        else:
            print(f'{node.check} 的 Camera 參數跟 topic 一致(基線 {baseline:.5f} m,來源 {source})')
    else:
        print(f'# 由 {node.left_topic} / {node.right_topic} 產生,基線 {baseline:.5f} m(來源:{source})')
        print('Camera:')
        print('  setup: "stereo"')
        print('  model: "perspective"')
        for key, value in cam.items():
            print(f'  {key}: {value}')
        print('  color_order: "RGB"  # 依影像 encoding 調整(rgb8 -> RGB,bgr8 -> BGR)')
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
