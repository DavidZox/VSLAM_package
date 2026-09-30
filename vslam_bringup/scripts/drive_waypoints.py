#!/usr/bin/env python3
"""VSLAM 測試用的簡單巡航:依序「原地轉向 -> 直線開到航點」,直接發 /cmd_vel,用 /odom 當回授。

刻意不走 Nav2 的 navigate_to_pose:這台機器人的 controller_server 有多個 goal checker,預設 BT 沒指定會
直接 abort(平常由 amr_base_sw 自己的 LocalNav/DockingLocalPlanner 指定 id 呼叫)。這支只拿來在模擬裡
沿路網車道重複繞同一條路線測 VSLAM,不做避障——航點請選路網上的節點(車道是淨空的),而且機器人端的
導航不要同時在發 /cmd_vel。

航點用 map 座標,啟動時從 TF 查一次 map -> odom 換到 odom 座標(查不到就當兩者重合並警告)。
預設是 fih_humanoid_amrScene 路網 a7 -> a6 -> a5 -> a8 -> a7 約 3 x 2.6 m 的矩形迴圈,最後回到起點
(0, 0) 轉回朝 +x,回到原點可以看 VSLAM 的 loop closure。

用法:
  ros2 run vslam_bringup drive_waypoints.py --ros-args -p laps:=1
  ros2 run vslam_bringup drive_waypoints.py --ros-args -p waypoints:="1.0,0.0;1.0,1.0" -p final_yaw_deg:=nan
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

DEFAULT_WAYPOINTS = '2.75,-0.45;2.75,1.76;-0.28,2.12;-0.29,-0.45;0.0,0.0'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class WaypointDriver(Node):
    def __init__(self):
        super().__init__('drive_waypoints')
        text = self.declare_parameter('waypoints', DEFAULT_WAYPOINTS).value
        self.waypoints = [tuple(float(v) for v in item.split(',')[:2]) for item in text.split(';') if item.strip()]
        self.laps = int(self.declare_parameter('laps', 1).value)
        self.final_yaw = math.radians(float(self.declare_parameter('final_yaw_deg', 0.0).value))
        self.linear_speed = float(self.declare_parameter('linear_speed', 0.3).value)
        self.angular_speed = float(self.declare_parameter('angular_speed', 0.5).value)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.pub = self.create_publisher(Twist, self.declare_parameter('cmd_vel_topic', '/cmd_vel').value, 10)
        self.create_subscription(Odometry, self.declare_parameter('odom_topic', '/odom').value, self.on_odom, 10)
        self.pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def map_to_odom(self):
        """回傳把 map 座標換成 odom 座標的函式。"""
        end = time.monotonic() + 5.0
        while time.monotonic() < end and not self.tf_buffer.can_transform(self.odom_frame, self.map_frame, Time()):
            rclpy.spin_once(self, timeout_sec=0.05)
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.map_frame, Time())
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            yaw = yaw_of(tf.transform.rotation)
            self.get_logger().info(f'航點 map 座標換到 odom:平移 ({tx:.3f}, {ty:.3f}),旋轉 {math.degrees(yaw):.2f} 度')
        except TransformException as ex:
            self.get_logger().warn(f'查不到 {self.map_frame} -> {self.odom_frame},航點直接當 odom 座標用: {ex}')
            tx = ty = yaw = 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        return lambda x, y, th=None: (c * x - s * y + tx, s * x + c * y + ty, None if th is None else th + yaw)

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def rotate_to(self, target_yaw):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            err = wrap(target_yaw - self.pose[2])
            if abs(err) < math.radians(1.5):
                break
            w = max(min(1.5 * err, self.angular_speed), -self.angular_speed)
            if abs(w) < 0.08:
                w = math.copysign(0.08, err)
            self.send(0.0, w)
        self.send(0.0, 0.0)
        self.spin_for(0.5)

    def drive_to(self, gx, gy):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            x, y, yaw = self.pose
            dist = math.hypot(gx - x, gy - y)
            if dist < 0.04:
                break
            heading_err = wrap(math.atan2(gy - y, gx - x) - yaw)
            if abs(heading_err) > math.radians(30):
                self.rotate_to(math.atan2(gy - y, gx - x))
                continue
            v = min(self.linear_speed, 0.8 * dist + 0.03)
            w = max(min(2.0 * heading_err, self.angular_speed), -self.angular_speed)
            self.send(v, w)
        self.send(0.0, 0.0)
        self.spin_for(0.5)

    def run(self):
        start = time.monotonic()
        while self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > 10.0:
                self.get_logger().error('10 秒內沒收到 odom,模擬有在 play 嗎?')
                return False
        to_odom = self.map_to_odom()
        for lap in range(self.laps):
            for i, (mx, my) in enumerate(self.waypoints):
                gx, gy, _ = to_odom(mx, my)
                x, y, _ = self.pose
                self.get_logger().info(f'第 {lap + 1}/{self.laps} 圈,航點 {i + 1}/{len(self.waypoints)}: '
                                       f'map ({mx:.2f}, {my:.2f}),距離 {math.hypot(gx - x, gy - y):.2f} m')
                self.rotate_to(math.atan2(gy - y, gx - x))
                self.drive_to(gx, gy)
        if not math.isnan(self.final_yaw):
            self.rotate_to(to_odom(0.0, 0.0, self.final_yaw)[2])
        self.get_logger().info('全部航點完成')
        return True


def main():
    rclpy.init()
    node = WaypointDriver()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
