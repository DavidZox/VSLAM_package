#!/usr/bin/env python3
"""把 VSLAM 估計的相機軌跡跟 ground truth 用同一組時間戳記錄成兩份 TUM 檔,給 evaluate_trajectory.py 算誤差。

ground truth 來源(gt_source 參數):
  odom(預設) 訂閱 gt_odom_topic(Isaac Sim 的 /odom 是底盤剛體的真實位姿),再乘上「底盤 -> 相機」外參。
             外參在第一次需要時從 TF 查一次後快取(頭部關節不動時是定值),每筆資料不用再查 TF。
  tf         每筆 VSLAM 位姿都用它的時間戳查 TF(gt_parent_frame -> 相機 frame)。這台機器人手指關節很多,
             /tf 流量大(約 144 則/秒),Python 的 TF listener 會落後,所以不是預設。
實機上 odom 會漂,只能當參考,不是真值。

輸出(output_dir 底下):
  est_tum.txt   VSLAM 位姿(VSLAM 訊息的 header.frame_id 座標系,例如 vslam_map)
  gt_tum.txt    ground truth(odom 座標系)
兩份檔案逐行時間戳一致,Ctrl+C 結束時會印出筆數。

用法:
  ros2 run vslam_bringup vslam_trajectory_recorder.py --ros-args -p output_dir:=/tmp/vslam_eval -p use_sim_time:=true
"""
import bisect
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def pose_to_matrix(position, orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [position.x, position.y, position.z]
    return T


def matrix_to_tum(t, T):
    R = T[:3, :3]
    # 旋轉矩陣轉四元數(Shepperd 方法,數值穩定)
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    p = T[:3, 3]
    return f'{t:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {x:.9f} {y:.9f} {z:.9f} {w:.9f}\n'


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('vslam_trajectory_recorder')
        self.pose_topic = self.declare_parameter('pose_topic', '/run_slam/camera_pose').value
        self.gt_source = self.declare_parameter('gt_source', 'odom').value
        self.gt_odom_topic = self.declare_parameter('gt_odom_topic', '/odom').value
        self.gt_parent_frame = self.declare_parameter('gt_parent_frame', 'odom').value
        # 空字串 = 用 VSLAM 訊息的 child_frame_id(相機 frame),兩邊比的是同一個剛體
        self.gt_child_frame = self.declare_parameter('gt_child_frame', '').value
        self.max_dt = float(self.declare_parameter('max_dt', 0.01).value)
        self.output_dir = self.declare_parameter('output_dir', '/tmp/vslam_eval').value

        os.makedirs(self.output_dir, exist_ok=True)
        self.est_file = open(os.path.join(self.output_dir, 'est_tum.txt'), 'w')
        self.gt_file = open(os.path.join(self.output_dir, 'gt_tum.txt'), 'w')
        self.count = 0
        self.failures = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_t, self.odom_T = [], []
        self.base_frame = None
        self.T_base_cam = None
        # VSLAM 位姿先排隊,等 ground truth 資料到了再配對(不在 callback 裡阻塞等 TF)
        self.pending = []
        if self.gt_source == 'odom':
            self.create_subscription(Odometry, self.gt_odom_topic, self.on_odom, 200)
        self.create_subscription(Odometry, self.pose_topic, self.on_pose, 200)
        self.create_timer(0.05, self.process_pending)
        self.get_logger().info(f'記錄 {self.pose_topic} 與 ground truth({self.gt_source}),輸出到 {self.output_dir}')

    def on_odom(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.base_frame = msg.child_frame_id
        if self.odom_t and t <= self.odom_t[-1]:
            return
        self.odom_t.append(t)
        self.odom_T.append(pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation))
        if len(self.odom_t) > 6000:
            del self.odom_t[:1000]
            del self.odom_T[:1000]

    def gt_from_odom(self, t, cam_frame):
        """回傳 ground truth;資料還沒到回傳 None(之後再試),確定拿不到丟 LookupError。"""
        if self.base_frame is None or self.odom_t[-1] < t:
            return None
        if self.T_base_cam is None:
            if not self.tf_buffer.can_transform(self.base_frame, cam_frame, Time()):
                return None
            tf = self.tf_buffer.lookup_transform(self.base_frame, cam_frame, Time())
            self.T_base_cam = pose_to_matrix(tf.transform.translation, tf.transform.rotation)
            p = self.T_base_cam[:3, 3]
            self.get_logger().info(f'外參 {self.base_frame} -> {cam_frame}: ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})')
        i = bisect.bisect_left(self.odom_t, t)
        best = min((k for k in (i - 1, i) if 0 <= k < len(self.odom_t)),
                   key=lambda k: abs(self.odom_t[k] - t), default=None)
        if best is None or abs(self.odom_t[best] - t) > self.max_dt:
            raise LookupError(f'odom 裡找不到時間 {t:.3f} 附近的資料')
        return self.odom_T[best] @ self.T_base_cam

    def gt_from_tf(self, stamp, cam_frame):
        if not self.tf_buffer.can_transform(self.gt_parent_frame, cam_frame, stamp):
            return None
        tf = self.tf_buffer.lookup_transform(self.gt_parent_frame, cam_frame, stamp)
        return pose_to_matrix(tf.transform.translation, tf.transform.rotation)

    def on_pose(self, msg: Odometry):
        self.pending.append((time.monotonic(), msg))

    def process_pending(self):
        while self.pending:
            arrived, msg = self.pending[0]
            cam_frame = self.gt_child_frame or msg.child_frame_id
            stamp = Time.from_msg(msg.header.stamp)
            t = stamp.nanoseconds * 1e-9
            try:
                T_gt = self.gt_from_odom(t, cam_frame) if self.gt_source == 'odom' else self.gt_from_tf(stamp, cam_frame)
                if T_gt is None and time.monotonic() - arrived > 3.0:
                    raise LookupError(f'等了 3 秒還拿不到時間 {t:.3f} 的 ground truth')
            except (TransformException, LookupError) as ex:
                self.pending.pop(0)
                self.failures += 1
                if self.failures % 20 == 1:
                    self.get_logger().warn(f'取不到 ground truth(累計 {self.failures} 次): {ex}')
                continue
            if T_gt is None:
                return
            self.pending.pop(0)
            self.est_file.write(matrix_to_tum(t, pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)))
            self.gt_file.write(matrix_to_tum(t, T_gt))
            self.count += 1
            if self.count % 200 == 0:
                self.est_file.flush()
                self.gt_file.flush()
                self.get_logger().info(f'已記錄 {self.count} 筆')

    def close(self):
        self.est_file.close()
        self.gt_file.close()
        self.get_logger().info(f'結束:共 {self.count} 筆,取不到真值 {self.failures} 次,檔案在 {self.output_dir}')


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
