#!/usr/bin/env python3
"""用雙目視差自動產生「機器人本體遮罩」PNG,給 run_slam --mask 用(0 = 不抽特徵,255 = 正常)。

為什麼需要:這台機器人頭上的相機看得到自己的雙手。手跟著相機一起動,在影像裡位置固定、特徵又多,
場景牆面卻是素色的,VSLAM 會被手上的特徵拉住,以為相機沒在動(2026-09-30 實測:不遮罩時整圈 13 m 的路線
估出來只剩 0.4 m 範圍,Sim3 尺度 2.9)。

做法:對 frames 組同時間戳的左右影像跑 SGBM,左右兩個參考視角各算一次,深度小於 max_depth 的像素算「近」。
在至少 min_ratio 比例的幀裡都很近的像素才當成機器人本體(過濾偶發的誤匹配、路過的近物),左右結果取聯集
(stella_vslam 左右影像用同一張遮罩),再做閉運算補洞、去掉小碎塊、往外膨脹 dilate_px 留邊。
機器人前方 max_depth 內不要有其他東西;可以邊移動邊拍,只有本體會一直很近。
手臂姿勢改了就要重拍。

用法(相機 topic 有在發的時候):
  ros2 run vslam_bringup make_self_mask.py --ros-args \\
    -p output:=vslam_bringup/config/isaac_zedx_self_mask.png -p focal_x_baseline:=58.82111
會另外輸出 <output 去掉副檔名>_preview.jpg 可以肉眼檢查。
"""
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def to_gray(msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'不支援的影像編碼 {msg.encoding}')
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
    img = img.reshape(msg.height, msg.width, channels) if channels > 1 else img.reshape(msg.height, msg.width)
    code = {'rgb8': cv2.COLOR_RGB2GRAY, 'bgr8': cv2.COLOR_BGR2GRAY,
            'rgba8': cv2.COLOR_RGBA2GRAY, 'bgra8': cv2.COLOR_BGRA2GRAY}.get(msg.encoding)
    return (cv2.cvtColor(img, code) if code is not None else img.copy()), img


class SelfMaskMaker(Node):
    def __init__(self):
        super().__init__('make_self_mask')
        self.left_topic = self.declare_parameter('left_image_topic', '/zed_camera_left/image_raw').value
        self.right_topic = self.declare_parameter('right_image_topic', '/zed_camera_right/image_raw').value
        self.output = self.declare_parameter('output', 'self_mask.png').value
        self.focal_x_baseline = float(self.declare_parameter('focal_x_baseline', 58.82111).value)
        self.max_depth = float(self.declare_parameter('max_depth', 1.0).value)
        self.frames = int(self.declare_parameter('frames', 20).value)
        self.min_interval = float(self.declare_parameter('min_interval', 0.5).value)
        self.min_ratio = float(self.declare_parameter('min_ratio', 0.6).value)
        self.dilate_px = int(self.declare_parameter('dilate_px', 15).value)
        self.min_blob_px = int(self.declare_parameter('min_blob_px', 400).value)
        self.left, self.right = {}, {}
        self.last_used = None
        self.count = None
        self.used = 0
        self.preview = None
        # 近到 max_depth 的視差要搜得到:numDisparities 取 16 的倍數,至少涵蓋 0.3 m
        self.num_disp = int(np.ceil(self.focal_x_baseline / 0.3 / 16.0)) * 16
        self.min_disp = self.focal_x_baseline / self.max_depth
        self.sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=self.num_disp, blockSize=7,
                                          P1=8 * 49, P2=32 * 49, uniquenessRatio=10,
                                          speckleWindowSize=100, speckleRange=2,
                                          mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.create_subscription(Image, self.left_topic, lambda m: self.on_image(m, self.left), 5)
        self.create_subscription(Image, self.right_topic, lambda m: self.on_image(m, self.right), 5)
        self.get_logger().info(f'收集 {self.frames} 組左右影像,深度 < {self.max_depth} m(視差 > {self.min_disp:.1f} px)'
                               '當作機器人本體')

    def on_image(self, msg, store):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        store[stamp] = msg
        for d in (self.left, self.right):
            for k in [k for k in d if k < stamp - 1.0]:
                del d[k]
        common = sorted(set(self.left) & set(self.right))
        if not common:
            return
        t = common[-1]
        if self.last_used is not None and t - self.last_used < self.min_interval:
            return
        self.last_used = t
        self.process(self.left.pop(t), self.right.pop(t))

    def process(self, left_msg, right_msg):
        gl, color = to_gray(left_msg)
        gr, _ = to_gray(right_msg)
        disp_l = self.sgbm.compute(gl, gr).astype(np.float32) / 16.0
        disp_r = cv2.flip(self.sgbm.compute(cv2.flip(gr, 1), cv2.flip(gl, 1)).astype(np.float32) / 16.0, 1)
        near = (disp_l > self.min_disp).astype(np.uint16) + (disp_r > self.min_disp).astype(np.uint16) * 256
        if self.count is None:
            self.count = np.zeros(gl.shape, np.uint16), np.zeros(gl.shape, np.uint16)
            self.preview = color if color.ndim == 3 else cv2.cvtColor(color, cv2.COLOR_GRAY2BGR)
            if left_msg.encoding.startswith('rgb'):
                self.preview = cv2.cvtColor(self.preview, cv2.COLOR_RGB2BGR)
        self.count[0][:] += (near & 0xFF) > 0
        self.count[1][:] += (near >> 8) > 0
        self.used += 1
        self.get_logger().info(f'第 {self.used}/{self.frames} 組')

    def finish(self):
        need = max(1, int(np.ceil(self.min_ratio * self.used)))
        body = ((self.count[0] >= need) | (self.count[1] >= need)).astype(np.uint8) * 255
        body = cv2.morphologyEx(body, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
        num, labels, stats, _ = cv2.connectedComponentsWithStats(body, connectivity=8)
        for i in range(1, num):
            if stats[i, cv2.CC_STAT_AREA] < self.min_blob_px:
                body[labels == i] = 0
        if self.dilate_px > 0:
            k = 2 * self.dilate_px + 1
            body = cv2.dilate(body, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        mask = np.where(body > 0, 0, 255).astype(np.uint8)
        os.makedirs(os.path.dirname(os.path.abspath(self.output)), exist_ok=True)
        cv2.imwrite(self.output, mask)
        preview = self.preview.copy()
        preview[mask == 0] = (0.4 * preview[mask == 0] + np.array([0, 0, 150])).astype(np.uint8)
        preview_path = os.path.splitext(self.output)[0] + '_preview.jpg'
        cv2.imwrite(preview_path, preview)
        self.get_logger().info(f'遮掉 {100.0 * (mask == 0).mean():.1f}% 的像素,存到 {self.output}(預覽 {preview_path})')


def main():
    rclpy.init()
    node = SelfMaskMaker()
    try:
        while rclpy.ok() and node.used < node.frames:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.finish()
    except KeyboardInterrupt:
        if node.used:
            node.finish()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
