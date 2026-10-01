#!/usr/bin/env python3
"""把 vslam_viz_recorder.py 錄的資料做成影片。離線執行,不需要 ROS(numpy + OpenCV + Pillow)。

輸出(--out-dir,預設 <錄製目錄>/videos):
  camera.mp4         機器人相機視角:原始左影像 + 演算法抽到/追蹤到的特徵點
  topdown.mp4        俯視全域地圖(VSLAM 地圖座標系)
                       建圖:地圖點一路累積、關鍵幀軌跡,迴環偵測後整段軌跡跟地圖被修正的過程
                       定位:當前影格特徵點用雙目深度轉成 3D、再用估計位姿放進地圖,看它們跟既有地圖點對不對得上
  side_by_side.mp4   俯視圖 + 相機視角 + 狀態面板並排
  topdown_final.png  最後一幀俯視圖

時間軸用模擬時間:--speed 1 = 機器人實際速度(Isaac Sim 跑得比真實時間慢,錄的時候不用管)。
真值(Isaac /odom)跟機器人的 2D 佔據地圖用錄到的 TF 換到 VSLAM 地圖座標系:
  odom -> vslam_map  run_slam 發的靜態 TF;純定位時用 --anchor-tf 指定建圖那次的 tf.txt,誤差才是相對地圖的絕對誤差
  map -> odom        AMCL;AMCL 沒定位好時用 --map-to-odom identity(把 odom 當成地圖座標系)

用法:
  python3 render_vslam_video.py datasets/isaac_eval/run9_mapping --mode mapping
  python3 render_vslam_video.py datasets/isaac_eval/run10_localization --mode localization \\
      --anchor-tf datasets/isaac_eval/run9_mapping/tf.txt
"""
import argparse
import bisect
import glob
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# 顏色(BGR)。相機畫面跟俯視圖的特徵點用同一套:綠 = 對到地圖點,橘 = 沒對到
GREEN = (0, 190, 0)
ORANGE = (0, 140, 255)
RED = (40, 40, 220)
BLUE = (200, 110, 20)
MAGENTA = (200, 0, 200)
BLACK = (0, 0, 0)
WALL_POINT = (80, 80, 80)
FLOOR_POINT = (190, 190, 190)
OCCUPIED = (222, 222, 222)
UNKNOWN = (246, 246, 246)
GRID = (238, 238, 238)
HEADER_BG = (40, 40, 40)
FOOTER_BG = (250, 250, 250)

FONT_CANDIDATES = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-{w}.ttc',
    '/usr/local/share/fonts/host/opentype/noto/NotoSansCJK-{w}.ttc',
    '/usr/share/fonts/noto-cjk/NotoSansCJK-{w}.ttc',
]


# ---------------------------------------------------------------- 資料

def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_matrix(v):
    """x y z qx qy qz qw -> 4x4"""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(*v[3:7])
    T[:3, 3] = v[:3]
    return T


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def load_tum(path):
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return np.zeros((0, 8))
    data = np.loadtxt(path, ndmin=2)
    return data[np.argsort(data[:, 0], kind='stable')]


def list_stamped(directory, ext):
    """<ns>.<ext> 檔案 -> (時間戳[秒] 陣列, 路徑清單),依時間排序"""
    if not os.path.isdir(directory):
        return np.zeros(0), []
    names = sorted(f for f in os.listdir(directory) if f.endswith(ext))
    return np.array([int(f[:-len(ext)]) * 1e-9 for f in names]), [os.path.join(directory, f) for f in names]


def load_tf(path):
    """tf.txt -> {(parent, child): [(t, 4x4), ...]}"""
    tf = {}
    if path and os.path.isfile(path):
        for line in open(path):
            p = line.split()
            if len(p) == 10:
                tf.setdefault((p[1], p[2]), []).append((float(p[0]), pose_matrix(np.array(p[3:], float))))
    return tf


class Recording:
    def __init__(self, directory, args):
        self.dir = directory
        self.camera_t, self.camera_files = list_stamped(os.path.join(directory, 'camera'), '.jpg')
        self.fp_t, self.fp_files = list_stamped(os.path.join(directory, 'frame_points'), '.npy')
        self.mp_t, self.mp_files = list_stamped(os.path.join(directory, 'map_points'), '.npy')
        self.kf_t, self.kf_files = list_stamped(os.path.join(directory, 'keyframes'), '.npy')
        self.le_t, self.le_files = list_stamped(os.path.join(directory, 'loop_edges'), '.npy')
        self.robot = load_tum(os.path.join(directory, 'robot_pose.txt'))
        self.camera_pose = load_tum(os.path.join(directory, 'camera_pose.txt'))
        self.gt = load_tum(os.path.join(directory, 'gt_odom.txt'))
        self.states = []
        state_path = os.path.join(directory, 'tracking_state.txt')
        if os.path.isfile(state_path):
            for line in open(state_path):
                p = line.split()
                if len(p) == 2:
                    self.states.append((float(p[0]), p[1]))
        self.states.sort()
        if len(self.camera_t) == 0:
            sys.exit(f'{directory}/camera 沒有影像')

        # odom -> VSLAM 地圖(靜態);沒有就用第一筆同時間戳的真值/VSLAM 位姿對齊
        tf = load_tf(os.path.join(directory, 'tf.txt'))
        anchor = load_tf(args.anchor_tf) if args.anchor_tf else tf
        anchors = [v for (parent, child), v in anchor.items() if parent == args.odom_frame and child == args.map_frame]
        if anchors:
            self.T_odom_map = anchors[0][-1][1]
        elif len(self.gt) and len(self.robot):
            t0 = self.robot[0, 0]
            T_odom_base = pose_matrix(self.gt[min(np.searchsorted(self.gt[:, 0], t0), len(self.gt) - 1), 1:])
            self.T_odom_map = T_odom_base @ np.linalg.inv(pose_matrix(self.robot[0, 1:]))
            print('沒有 odom -> VSLAM 地圖的靜態 TF,用第一筆位姿對齊真值', file=sys.stderr)
        else:
            self.T_odom_map = np.eye(4)
        T_map_odom = np.eye(4)
        if args.map_to_odom == 'tf':
            amcl = tf.get(('map', args.odom_frame), [])
            if amcl:
                T_map_odom = amcl[len(amcl) // 2][1]
            else:
                print('沒錄到 map -> odom,佔據地圖當成跟 odom 重合', file=sys.stderr)
        # 佔據地圖(機器人的 map 座標系)-> VSLAM 地圖座標系
        self.T_vslam_occ = np.linalg.inv(self.T_odom_map) @ np.linalg.inv(T_map_odom)
        self.occupancy = None
        occ_path = os.path.join(directory, 'occupancy.npz')
        if not args.no_occupancy and os.path.isfile(occ_path):
            self.occupancy = np.load(occ_path)

        # 真值換到 VSLAM 地圖座標系(底盤位置 + yaw)
        self.gt_xy = np.zeros((0, 2))
        if len(self.gt):
            T_map_odom_gt = np.linalg.inv(self.T_odom_map)
            xyz = (T_map_odom_gt[:3, :3] @ self.gt[:, 1:4].T).T + T_map_odom_gt[:3, 3]
            self.gt_xy = xyz[:, :2]

        # 關鍵幀是相機位姿,畫軌跡要換成底盤:T(相機 <- 底盤) 是固定的(頭部關節不動),
        # 用同時間戳的 camera_pose / robot_pose 求
        self.T_cam_base = None
        if len(self.camera_pose) and len(self.robot):
            idx = np.searchsorted(self.camera_pose[:, 0], self.robot[:, 0])
            for i, j in enumerate(idx):
                if j < len(self.camera_pose) and abs(self.camera_pose[j, 0] - self.robot[i, 0]) < 1e-6:
                    self.T_cam_base = np.linalg.inv(pose_matrix(self.camera_pose[j, 1:])) @ pose_matrix(self.robot[i, 1:])
                    break

    @staticmethod
    def latest(stamps, t):
        """stamps 裡 <= t 的最後一個 index,沒有就 -1"""
        return bisect.bisect_right(stamps, t + 1e-9) - 1

    def state_at(self, t):
        state = 'Initializing'
        for ts, s in self.states:
            if ts <= t + 1e-6:
                state = s
            else:
                break
        return state

    def keyframe_base_xy(self, index):
        if index < 0:
            return np.zeros((0, 2)), np.zeros((0, 3))
        kf = np.load(self.kf_files[index])
        if len(kf) == 0:
            return np.zeros((0, 2)), np.zeros((0, 3))
        cam_xyz = kf[:, :3]
        if self.T_cam_base is None:
            return cam_xyz[:, :2], cam_xyz
        base = np.array([(pose_matrix(row) @ self.T_cam_base)[:2, 3] for row in kf])
        return base, cam_xyz


# ---------------------------------------------------------------- 字型/文字

def find_font(weight, override=None):
    if override:
        return override
    for pattern in FONT_CANDIDATES:
        path = pattern.format(w=weight)
        if os.path.isfile(path):
            return path
    found = sorted(glob.glob(f'/usr/**/NotoSansCJK*{weight}*.tt[cf]', recursive=True))
    if found:
        return found[0]
    sys.exit('找不到中文字型(Noto Sans CJK):請安裝 fonts-noto-cjk,或用 --font 指定 .ttc/.ttf 檔')


class Fonts:
    def __init__(self, regular, bold):
        self.regular_path, self.bold_path = regular, bold
        self.index = {}
        self.cache = {}

    def _tc_index(self, path):
        # Noto Sans CJK 的 .ttc 有 JP/KR/SC/TC/HK 好幾個字面,選繁中(TC)
        if path not in self.index:
            self.index[path] = 0
            if path.endswith('.ttc'):
                for i in range(10):
                    try:
                        name = ImageFont.truetype(path, 12, index=i).getname()[0]
                    except OSError:
                        break
                    if name.endswith(' TC'):
                        self.index[path] = i
                        break
        return self.index[path]

    def get(self, size, bold=False):
        key = (size, bold)
        if key not in self.cache:
            path = self.bold_path if bold else self.regular_path
            self.cache[key] = ImageFont.truetype(path, size, index=self._tc_index(path))
        return self.cache[key]


def draw_texts(img, items):
    """items: [(x, y, 文字, font, BGR 顏色, anchor)],一次轉 PIL 畫完再轉回來"""
    if not items:
        return img
    pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    for x, y, text, font, color, anchor in items:
        draw.text((x, y), text, font=font, fill=(color[2], color[1], color[0]), anchor=anchor)
    return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


# ---------------------------------------------------------------- 俯視圖

class TopDownView:
    HEADER = 96
    FOOTER = 84

    def __init__(self, rec, args, size):
        self.rec, self.args = rec, args
        self.W, self.H = size
        self.map_top, self.map_bottom = self.HEADER, self.H - self.FOOTER
        x0, x1, y0, y1 = self.bounds(args.view)
        pad = 12
        avail_w, avail_h = self.W - 2 * pad, self.map_bottom - self.map_top - 2 * pad
        self.scale = min(avail_w / (x1 - x0), avail_h / (y1 - y0))
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        # 世界(VSLAM 地圖,x 右 y 上)-> 畫面像素
        self.ox = self.W / 2 - cx * self.scale
        self.oy = (self.map_top + self.map_bottom) / 2 + cy * self.scale
        self.base = self.make_base()
        self.map_cache = (None, None)

    def bounds(self, view):
        if view:
            return [float(v) for v in view.split(',')]
        # 軌跡外框再往外 view_margin 公尺:涵蓋附近的牆;更遠的地圖點(雙目深度誤差大)切掉,畫面才不會縮太小
        rec = self.rec
        xy = [traj for traj in (rec.robot[:, 1:3], rec.gt_xy) if len(traj)]
        if not xy:
            return -5, 5, -5, 5
        xy = np.vstack(xy)
        m = self.args.view_margin
        return xy[:, 0].min() - m, xy[:, 0].max() + m, xy[:, 1].min() - m, xy[:, 1].max() + m

    def to_px(self, xy):
        xy = np.asarray(xy, float).reshape(-1, 2)
        return np.c_[self.ox + xy[:, 0] * self.scale, self.oy - xy[:, 1] * self.scale]

    def make_base(self):
        img = np.full((self.H, self.W, 3), 255, np.uint8)
        occ = self.rec.occupancy
        if occ is not None:
            data = occ['data']
            res = float(occ['resolution'])
            origin = occ['origin']
            # 佔據地圖像素 (col, row) -> 地圖座標 -> VSLAM 地圖座標 -> 畫面像素 的 2D 仿射
            T_cell = np.eye(4)
            T_cell[:3, :3] = quat_to_matrix(*origin[3:7])
            T_cell[:3, 3] = origin[:3]
            T = self.rec.T_vslam_occ @ T_cell
            A = np.array([[T[0, 0] * res, T[0, 1] * res, T[0, 3] + (T[0, 0] + T[0, 1]) * res / 2],
                          [T[1, 0] * res, T[1, 1] * res, T[1, 3] + (T[1, 0] + T[1, 1]) * res / 2]])
            S = np.array([[self.scale, 0, self.ox], [0, -self.scale, self.oy]])
            M = S @ np.vstack([A, [0, 0, 1]])
            layer = np.full(data.shape, 255, np.uint8)
            layer[data < 0] = 1
            layer[data > 50] = 2
            warped = cv2.warpAffine(layer, M, (self.W, self.H), flags=cv2.INTER_NEAREST, borderValue=0)
            img[warped == 1] = UNKNOWN
            img[warped == 2] = OCCUPIED
        # 1 m 格線
        x_min = math.floor((0 - self.ox) / self.scale)
        x_max = math.ceil((self.W - self.ox) / self.scale)
        y_min = math.floor((self.oy - self.map_bottom) / self.scale)
        y_max = math.ceil((self.oy - self.map_top) / self.scale)
        for x in range(x_min, x_max + 1):
            u = int(round(self.ox + x * self.scale))
            cv2.line(img, (u, self.map_top), (u, self.map_bottom), GRID, 1)
        for y in range(y_min, y_max + 1):
            v = int(round(self.oy - y * self.scale))
            cv2.line(img, (0, v), (self.W, v), GRID, 1)
        img[:self.map_top] = HEADER_BG
        img[self.map_bottom:] = FOOTER_BG
        # 比例尺
        u0, v0 = 20, self.map_bottom - 20
        cv2.line(img, (u0, v0), (u0 + int(self.scale), v0), BLACK, 3)
        cv2.line(img, (u0, v0 - 6), (u0, v0 + 6), BLACK, 2)
        cv2.line(img, (u0 + int(self.scale), v0 - 6), (u0 + int(self.scale), v0 + 6), BLACK, 2)
        return img

    def points_px(self, xyz, size):
        """N x 2+ 點 -> 畫面內的整數像素(左上角),配合 size x size 的方塊"""
        p = self.to_px(xyz[:, :2])
        u = np.round(p[:, 0]).astype(int) - size // 2
        v = np.round(p[:, 1]).astype(int) - size // 2
        ok = (u >= 0) & (u < self.W - size) & (v >= self.map_top) & (v < self.map_bottom - size)
        return u, v, ok

    def splat(self, img, xyz, color, size):
        u, v, ok = self.points_px(xyz, size)
        u, v = u[ok], v[ok]
        for du in range(size):
            for dv in range(size):
                img[v + dv, u + du] = color

    def map_layer(self, index):
        if self.map_cache[0] == index:
            return self.map_cache[1]
        img = self.base.copy()
        n = 0
        if index >= 0:
            pts = np.load(self.rec.mp_files[index])
            n = len(pts)
            floor = pts[:, 2] < self.args.floor_height
            self.splat(img, pts[floor], FLOOR_POINT, 2)
            self.splat(img, pts[~floor], WALL_POINT, 2)
        self.map_cache = (index, (img, n))
        return img, n

    def polyline(self, img, xy, color, thickness, dashed=False):
        if len(xy) < 2:
            return
        p = np.round(self.to_px(xy)).astype(np.int32)
        if not dashed:
            cv2.polylines(img, [p], False, color, thickness, cv2.LINE_AA)
            return
        # 虛線:沿線每 12 px 畫 7 px
        seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
        dist = np.r_[0, np.cumsum(seg)]
        on = (dist % 12) < 7
        for i in range(len(p) - 1):
            if on[i]:
                cv2.line(img, tuple(p[i]), tuple(p[i + 1]), color, thickness, cv2.LINE_AA)

    def robot_marker(self, img, T, color, camera_offset):
        """底盤位姿 T(4x4,VSLAM 地圖):相機視野扇形 + 圓點"""
        yaw = yaw_of(T)
        fwd, left = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])
        center = T[:2, 3]
        # 相機視野(水平 FOV、畫到 fov_range 公尺),從相機實際位置(底盤座標的 camera_offset)開始
        half = math.radians(self.args.fov_deg) / 2
        cam = center + camera_offset[0] * fwd + camera_offset[1] * left
        arc = [cam + self.args.fov_range * np.array([math.cos(yaw + a), math.sin(yaw + a)])
               for a in np.linspace(-half, half, 24)]
        poly = np.round(self.to_px(np.vstack([cam, arc]))).astype(np.int32)
        overlay = img.copy()
        cv2.fillPoly(overlay, [poly], color)
        cv2.addWeighted(overlay, 0.12, img, 0.88, 0, dst=img)
        cv2.polylines(img, [poly], True, color, 1, cv2.LINE_AA)
        # 機器人:圓點 + 朝向線(方向看扇形;小三角形/箭頭縮小後容易看反)
        c = tuple(int(v) for v in np.round(self.to_px(center))[0])
        tip = tuple(int(v) for v in np.round(self.to_px(center + 0.2 * fwd))[0])
        radius = max(6, int(round(0.2 * self.scale)))
        cv2.circle(img, c, radius, color, -1, cv2.LINE_AA)
        cv2.circle(img, c, radius, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.line(img, c, tip, (255, 255, 255), 2, cv2.LINE_AA)

    def frame_points(self, img, fp):
        if fp is None or len(fp) == 0:
            return
        tracked = fp[:, 3] > 0.5
        for pts, color, radius in ((fp[~tracked], ORANGE, 2), (fp[tracked], GREEN, 3)):
            u, v, ok = self.points_px(pts, 0)
            for uu, vv in zip(u[ok], v[ok]):
                cv2.circle(img, (int(uu), int(vv)), radius, color, -1, cv2.LINE_AA)


# ---------------------------------------------------------------- 主流程

class Renderer:
    def __init__(self, rec, args, fonts):
        self.rec, self.args, self.fonts = rec, args, fonts
        self.mapping = args.mode == 'mapping'
        self.view = TopDownView(rec, args, (1080, 1080))
        # 迴環事件:迴環邊數量變多的時間(純定位時載入的地圖本來就有迴環邊,不算事件)
        self.loop_events = []
        prev = 0
        for t, path in zip(rec.le_t, rec.le_files) if self.mapping else ():
            n = len(np.load(path))
            if n > prev:
                self.loop_events.append((t, n))
            prev = n
        if self.loop_events:
            print('迴環偵測:', ', '.join(f't={t - rec.camera_t[0]:.1f}s(第 {n} 條)' for t, n in self.loop_events))
        # 進入 Tracking 的時間:第一次(純定位 = 在載入的地圖裡重定位成功)、Lost 之後(重新接上)
        self.first_tracking = next((t for t, s in rec.states if s == 'Tracking'), None)
        self.recoveries = [t for (t, s), (_, prev) in zip(rec.states[1:], rec.states[:-1])
                           if s == 'Tracking' and prev == 'Lost' and t != self.first_tracking]
        if self.recoveries:
            print('Lost 後重新追蹤:', ', '.join(f't={t - rec.camera_t[0]:.1f}s' for t in self.recoveries))
        # 相機在底盤座標的水平位置(畫視野扇形的起點)
        self.camera_offset = np.zeros(2)
        if rec.T_cam_base is not None:
            self.camera_offset = np.linalg.inv(rec.T_cam_base)[:2, 3]

    def timeline(self):
        rec, args = self.rec, self.args
        t0 = rec.camera_t[0] + args.start
        t1 = rec.camera_t[-1] if args.end is None else rec.camera_t[0] + args.end
        n = int(math.floor((t1 - t0) * args.fps / args.speed)) + 1
        times = [t0 + k * args.speed / args.fps for k in range(n)]
        hold = int(round(args.hold * args.fps))
        return times, hold

    def frame_data(self, t):
        rec = self.rec
        d = {'t': t, 'state': rec.state_at(t)}
        i = rec.latest(rec.camera_t, t)
        d['camera'] = rec.camera_files[max(i, 0)]
        i = rec.latest(rec.fp_t, t)
        d['fp'] = np.load(rec.fp_files[i]) if i >= 0 and t - rec.fp_t[i] < 0.2 else None
        d['map_index'] = rec.latest(rec.mp_t, t)
        d['kf_index'] = rec.latest(rec.kf_t, t)
        i = rec.latest(rec.le_t, t)
        d['loop_edges'] = np.load(rec.le_files[i]) if i >= 0 else np.zeros((0, 2, 3))
        n = bisect.bisect_right(rec.robot[:, 0], t + 1e-9) if len(rec.robot) else 0
        d['robot_traj'] = rec.robot[:n, 1:3]
        d['robot_T'] = pose_matrix(rec.robot[n - 1, 1:]) if n else None
        d['robot_fresh'] = n > 0 and t - rec.robot[n - 1, 0] < 0.3
        n_gt = bisect.bisect_right(rec.gt[:, 0], t + 1e-9) if len(rec.gt) else 0
        d['gt_traj'] = rec.gt_xy[:n_gt]
        d['error'] = None
        if d['robot_fresh'] and n_gt:
            d['error'] = float(np.linalg.norm(d['robot_T'][:2, 3] - rec.gt_xy[n_gt - 1]))
        return d

    def topdown(self, d, final=False):
        rec, view, f = self.rec, self.view, self.fonts
        img, n_map = view.map_layer(d['map_index'])
        img = img.copy()
        texts = []
        t_rel = d['t'] - rec.camera_t[0]

        # 真值、關鍵幀(建圖)、VSLAM 即時軌跡
        view.polyline(img, d['gt_traj'], BLACK, 2, dashed=True)
        kf_xy, kf_cam = rec.keyframe_base_xy(d['kf_index'])
        if self.mapping:
            # 即時位姿不會被事後修正,關鍵幀會:迴環修正前兩條重疊(藍蓋住紅),修正後紅線露出來 = 修正前的估計
            view.polyline(img, d['robot_traj'], RED, 2)
            view.polyline(img, kf_xy, BLUE, 3)
            for u, v in np.round(view.to_px(kf_xy)).astype(int):
                cv2.circle(img, (u, v), 3, BLUE, -1, cv2.LINE_AA)
        else:
            view.polyline(img, d['robot_traj'], RED, 2)

        # 迴環邊(端點是關鍵幀相機位置,換成同一個關鍵幀的底盤位置);純定位時是地圖本來就有的,不畫
        n_loops = len(d['loop_edges'])
        for edge in d['loop_edges'] if self.mapping else ():
            ends = []
            for cam in edge:
                if len(kf_cam):
                    j = int(np.argmin(np.linalg.norm(kf_cam - cam, axis=1)))
                    ends.append(kf_xy[j])
                else:
                    ends.append(cam[:2])
            view.polyline(img, np.array(ends), MAGENTA, 3)

        if not final:
            view.frame_points(img, d['fp'])
        if d['robot_T'] is not None:
            view.robot_marker(img, d['robot_T'], RED if d['robot_fresh'] else (150, 150, 150), self.camera_offset)

        # 標題列(兩行):標題 + 時間/狀態,下面一行是數字
        title = 'VSLAM 建圖:俯視全域地圖' if self.mapping else 'VSLAM 純定位:影像特徵點轉 3D、對齊既有地圖'
        texts.append((24, 32, title, f.get(28, True), (255, 255, 255), 'lm'))
        state = {'Tracking': '追蹤中', 'Lost': '追蹤失敗', 'Initializing': '初始化'}.get(d['state'], d['state'])
        texts.append((self.view.W - 24, 32, f't = {t_rel:5.1f} s   {state}', f.get(24, True),
                      (255, 255, 255) if d['state'] == 'Tracking' else (90, 90, 255), 'rm'))
        if self.mapping:
            info = [f'關鍵幀 {len(kf_xy)}', f'地圖點 {n_map:,}', f'迴環 {n_loops}']
        else:
            info = [f'地圖點 {n_map:,}']
            if d['fp'] is not None:
                info += [f'這幀特徵點轉 3D {len(d["fp"])} 個,對到地圖 {int((d["fp"][:, 3] > 0.5).sum())} 個']
        if d['error'] is not None:
            info += [f'跟真值差 {d["error"] * 100:.1f} cm']
        texts.append((24, 74, '      '.join(info), f.get(21), (215, 215, 215), 'lm'))

        # 事件橫幅
        banner = None
        if final:
            banner = (f'完成:關鍵幀 {len(kf_xy)}、地圖點 {n_map:,}、迴環 {n_loops}' if self.mapping
                      else '定位結束', BLUE)
        elif d['state'] == 'Lost':
            banner = ('追蹤失敗(Lost)', RED)
        else:
            for t_e, n in self.loop_events:
                if 0 <= d['t'] - t_e < self.args.event_seconds:
                    banner = ('偵測到迴環:關鍵幀軌跡(藍)跟地圖一起修正,露出來的紅線是修正前的估計', MAGENTA)
            if not self.mapping and self.first_tracking is not None and 0 <= d['t'] - self.first_tracking < 3.0:
                banner = ('在載入的地圖裡重定位成功', BLUE)
            for t_r in self.recoveries:
                if 0 <= d['t'] - t_r < 3.0:
                    banner = ('短暫追蹤失敗(Lost)後,在地圖裡重定位回來' if not self.mapping
                              else '短暫追蹤失敗(Lost)後重新接上', RED)
        if banner:
            text, color = banner
            font = f.get(24, True)
            w = font.getbbox(text)[2] + 40
            x0 = (self.view.W - w) // 2
            y0 = self.view.HEADER + 14
            cv2.rectangle(img, (x0, y0), (x0 + w, y0 + 44), (255, 255, 255), -1)
            cv2.rectangle(img, (x0, y0), (x0 + w, y0 + 44), color, 3)
            texts.append((self.view.W // 2, y0 + 22, text, font, color, 'mm'))

        # 比例尺、圖例(兩行:點 / 線)
        small = f.get(17)
        texts.append((28 + int(view.scale), view.map_bottom - 20, '1 m', small, BLACK, 'lm'))
        rows = [[(WALL_POINT, 'sq', '地圖點'), (FLOOR_POINT, 'sq', '地圖點(地面)'),
                 (GREEN, 'dot', '這幀特徵點轉 3D(對到地圖)'), (ORANGE, 'dot', '這幀特徵點轉 3D(沒對到)')],
                [(RED, 'line', 'VSLAM 估計位姿' if not self.mapping else 'VSLAM 即時位姿'), (BLACK, 'dash', '真值(Isaac /odom)')]]
        if rec.occupancy is not None:
            rows[0].append((OCCUPIED, 'sq', '機器人 2D 地圖'))
        if self.mapping:
            rows[1][1:1] = [(BLUE, 'line', '關鍵幀軌跡(會被修正)')]
            rows[1].append((MAGENTA, 'line', '迴環邊'))
        for r, items in enumerate(rows):
            y_legend = view.map_bottom + 24 + r * 36
            x = 20
            for color, kind, label in items:
                if kind == 'sq':
                    cv2.rectangle(img, (x, y_legend - 6), (x + 12, y_legend + 6), color, -1)
                    cv2.rectangle(img, (x, y_legend - 6), (x + 12, y_legend + 6), (150, 150, 150), 1)
                elif kind == 'dot':
                    cv2.circle(img, (x + 6, y_legend), 5, color, -1, cv2.LINE_AA)
                elif kind == 'line':
                    cv2.line(img, (x - 2, y_legend), (x + 16, y_legend), color, 3, cv2.LINE_AA)
                else:
                    cv2.line(img, (x - 2, y_legend), (x + 4, y_legend), color, 2)
                    cv2.line(img, (x + 9, y_legend), (x + 16, y_legend), color, 2)
                texts.append((x + 22, y_legend, label, small, (60, 60, 60), 'lm'))
                x += 22 + int(small.getlength(label)) + 26
        return draw_texts(img, texts)

    def camera(self, d):
        img = cv2.imread(d['camera'])
        h, w = img.shape[:2]
        bar = 60
        out = np.full((h + bar, w, 3), 250, np.uint8)
        out[:h] = img
        f = self.fonts.get(20)
        y = h + bar // 2
        cv2.circle(out, (24, y), 6, GREEN, -1, cv2.LINE_AA)
        cv2.circle(out, (330, y), 4, ORANGE, 1, cv2.LINE_AA)
        cv2.rectangle(out, (640, y - 9), (658, y + 9), (70, 70, 70), -1)
        cv2.rectangle(out, (640, y - 9), (658, y + 9), (255, 255, 255), 1)
        mode = '建圖' if self.mapping else '純定位'
        texts = [(38, y, '對到地圖點、正在追蹤的特徵', f, (40, 40, 40), 'lm'),
                 (342, y, '這幀抽到但沒對到地圖的特徵', f, (40, 40, 40), 'lm'),
                 (668, y, '遮罩:機器人雙手,不抽特徵', f, (40, 40, 40), 'lm'),
                 (w - 20, y, f'{mode}  t = {d["t"] - self.rec.camera_t[0]:5.1f} s', f, (40, 40, 40), 'rm')]
        return draw_texts(out, texts)

    def side_by_side(self, top, cam, d):
        out = np.full((1080, 1920, 3), 250, np.uint8)
        out[:, :1080] = top
        cam_w = 1920 - 1080
        cam_small = cv2.resize(cam, (cam_w, int(cam.shape[0] * cam_w / cam.shape[1])), interpolation=cv2.INTER_AREA)
        out[:cam_small.shape[0], 1080:] = cam_small
        cv2.line(out, (1080, 0), (1080, 1080), (180, 180, 180), 2)
        f = self.fonts
        y = cam_small.shape[0] + 40
        texts = [(1110, y, '機器人相機視角(左眼)', f.get(26, True), (30, 30, 30), 'lm')]
        lines = ['• 綠點:對到地圖點、正在追蹤的特徵', '• 橘點:這幀抽到但沒對到地圖的特徵',
                 '• 暗區:機器人雙手的遮罩(不抽特徵)']
        if self.mapping:
            lines += ['', '俯視圖:機器人邊走邊把特徵點加進地圖(灰點),', '藍線是關鍵幀軌跡;回到起點偵測到迴環後,',
                      '整段軌跡跟地圖一起被修正(露出來的紅線是', '修正前的即時估計)。黑虛線是真值。']
        else:
            lines += ['', '俯視圖:灰點是建圖時存下的地圖,綠/橘點是這一幀', '特徵點用雙目深度轉成的 3D 點,再用估計位姿放進地圖;',
                      '對得上牆面的灰點就代表定位正確。黑虛線是真值。']
        for i, line in enumerate(lines):
            texts.append((1110, y + 46 + i * 34, line, f.get(21), (50, 50, 50), 'lm'))
        return draw_texts(out, texts)

    def run(self):
        args = self.args
        os.makedirs(args.out_dir, exist_ok=True)
        times, hold = self.timeline()
        writers = {}

        def write(name, img):
            if name not in writers:
                path = os.path.join(args.out_dir, f'{name}.mp4')
                h, w = img.shape[:2]
                writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'avc1'), args.fps, (w, h))
                if not writer.isOpened():
                    print(f'{name}:這個 OpenCV 不能寫 H.264,改用 mp4v', file=sys.stderr)
                    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (w, h))
                writers[name] = writer
            writers[name].write(img)

        print(f'{len(times)} 幀 + 停留 {hold} 幀,{args.fps} fps,{args.speed}x 模擬時間 -> {args.out_dir}')
        top = cam = None
        for k, t in enumerate(times):
            d = self.frame_data(t)
            top = self.topdown(d)
            cam = self.camera(d)
            write('topdown', top)
            write('camera', cam)
            write('side_by_side', self.side_by_side(top, cam, d))
            if k % 300 == 0:
                print(f'  {k}/{len(times)}', flush=True)
        # 最後停幾秒:建圖顯示最後的完整地圖(最後一份地圖點/關鍵幀/迴環邊)
        d = self.frame_data(times[-1])
        d['map_index'], d['kf_index'] = len(self.rec.mp_files) - 1, len(self.rec.kf_files) - 1
        if self.rec.le_files:
            d['loop_edges'] = np.load(self.rec.le_files[-1])
        final = self.topdown(d, final=True)
        cv2.imwrite(os.path.join(args.out_dir, 'topdown_final.png'), final)
        for _ in range(hold):
            write('topdown', final)
            write('camera', cam)
            write('side_by_side', self.side_by_side(final, cam, d))
        for writer in writers.values():
            writer.release()
        print('完成:', ', '.join(sorted(os.listdir(args.out_dir))))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('record_dir')
    parser.add_argument('--mode', choices=['mapping', 'localization'], required=True)
    parser.add_argument('--out-dir', help='預設 <record_dir>/videos')
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--speed', type=float, default=1.0, help='每秒影片 = 幾秒模擬時間')
    parser.add_argument('--start', type=float, default=0.0, help='從第一張影像後幾秒(模擬時間)開始')
    parser.add_argument('--end', type=float, help='到第一張影像後幾秒結束(預設到最後)')
    parser.add_argument('--hold', type=float, default=4.0, help='最後一幀停留秒數')
    parser.add_argument('--event-seconds', type=float, default=4.0, help='迴環橫幅顯示幾秒(模擬時間)')
    parser.add_argument('--view', help='俯視範圍 x0,x1,y0,y1(VSLAM 地圖座標,公尺);預設 = 軌跡外框 + --view-margin')
    parser.add_argument('--view-margin', type=float, default=2.5, help='自動範圍:軌跡外框往外幾公尺')
    parser.add_argument('--floor-height', type=float, default=0.15, help='低於這個高度的地圖點當地面(淺灰)')
    parser.add_argument('--fov-deg', type=float, default=105.1, help='相機水平視角(畫扇形用)')
    parser.add_argument('--fov-range', type=float, default=2.5, help='扇形畫多長(公尺)')
    parser.add_argument('--map-frame', default='vslam_map')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--anchor-tf', help='odom -> VSLAM 地圖的 TF 從這個 tf.txt 拿(純定位時給建圖那次的)')
    parser.add_argument('--map-to-odom', choices=['tf', 'identity'], default='tf',
                        help='佔據地圖怎麼放:tf = 錄到的 AMCL map -> odom;identity = 當成跟 odom 重合')
    parser.add_argument('--no-occupancy', action='store_true', help='不畫 2D 佔據地圖底圖')
    parser.add_argument('--font', help='中文字型(.ttc/.ttf),預設找 Noto Sans CJK')
    args = parser.parse_args()
    args.out_dir = args.out_dir or os.path.join(args.record_dir, 'videos')
    fonts = Fonts(find_font('Regular', args.font), find_font('Bold', args.font))
    rec = Recording(args.record_dir, args)
    Renderer(rec, args, fonts).run()


if __name__ == '__main__':
    main()
