#!/usr/bin/env python3
"""比較 VSLAM 估計軌跡跟 ground truth(兩份 TUM 格式檔),只用 numpy,不需要 evo。

算的東西:
  ATE RMSE(SE3)   用 Umeyama 做剛體對齊(不縮放)後的位置誤差,看整體精度
  尺度 (Sim3)      允許縮放時估出來的尺度,雙目應該接近 1.0;明顯偏離代表基線/焦距設錯
  起點對齊漂移      只用第一個位姿對齊(不看未來),終點誤差 / 路徑長 = 即時定位會看到的累積漂移
  軌跡疊圖          --plot 給路徑時輸出 xy 平面疊圖 PNG

時間戳用最近鄰配對(預設容許 0.02 s);vslam_trajectory_recorder.py 錄的兩份檔時間戳本來就一致。
stella_vslam 的 --eval-log-dir 輸出的 frame_trajectory.txt 是「光學座標系(z 朝前)」的相機位姿,
拿它跟 recorder 的 gt_tum.txt(相機 link,x 朝前)比的時候加 --est-optical,起點對齊才會正確
(ATE 只用位置,不受影響)。

用法:
  evaluate_trajectory.py <gt_tum.txt> <est_tum.txt> [--est-optical] [--plot out.png] [--max-dt 0.02]
"""
import argparse
import sys

import numpy as np

# 光學座標系(x 右 y 下 z 前)轉到相機 link(x 前 y 左 z 上)的旋轉:T_link_optical 的旋轉部分
R_LINK_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])


def load_tum(path):
    data = np.loadtxt(path, comments='#', ndmin=2)
    if data.shape[1] != 8:
        raise ValueError(f'{path} 不是 TUM 格式(每行 8 欄:t x y z qx qy qz qw)')
    return data


def quat_to_rot(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def to_matrices(rows, optical=False):
    mats = []
    for r in rows:
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(r[4:8])
        if optical:
            T[:3, :3] = T[:3, :3] @ R_LINK_OPTICAL.T
        T[:3, 3] = r[1:4]
        mats.append(T)
    return np.array(mats)


def associate(gt, est, max_dt):
    idx_gt, idx_est = [], []
    gt_t = gt[:, 0]
    for j, t in enumerate(est[:, 0]):
        i = int(np.searchsorted(gt_t, t))
        best = None
        for k in (i - 1, i):
            if 0 <= k < len(gt_t) and (best is None or abs(gt_t[k] - t) < abs(gt_t[best] - t)):
                best = k
        if best is not None and abs(gt_t[best] - t) <= max_dt:
            idx_gt.append(best)
            idx_est.append(j)
    return np.array(idx_gt, dtype=int), np.array(idx_est, dtype=int)


def umeyama(src, dst, with_scale):
    """回傳 (s, R, t) 使 dst ~= s * R @ src + t。src/dst: N x 3。"""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / xs.var(axis=0).sum() if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gt')
    ap.add_argument('est')
    ap.add_argument('--est-optical', action='store_true', help='估計檔是光學座標系的相機位姿(stella 的 frame_trajectory.txt)')
    ap.add_argument('--max-dt', type=float, default=0.02, help='時間戳配對容許誤差(秒)')
    ap.add_argument('--align-from', nargs=2, metavar=('GT', 'EST'), default=None,
                    help='起點對齊改用另一次錄製的第一筆位姿(例如載入地圖定位時,用建圖那次的 gt/est),'
                         '量的是相對建圖座標系的絕對誤差')
    ap.add_argument('--plot', default='', help='輸出 xy 疊圖 PNG 路徑')
    ap.add_argument('--title', default='VSLAM vs ground truth')
    args = ap.parse_args()

    gt_rows, est_rows = load_tum(args.gt), load_tum(args.est)
    gt_rows = gt_rows[np.argsort(gt_rows[:, 0])]
    ig, ie = associate(gt_rows, est_rows, args.max_dt)
    if len(ig) < 3:
        print(f'配對成功只有 {len(ig)} 筆,無法評估(檢查時間戳是否同一個時鐘、--max-dt)', file=sys.stderr)
        return 1
    T_gt = to_matrices(gt_rows[ig])
    T_est = to_matrices(est_rows[ie], optical=args.est_optical)
    p_gt, p_est = T_gt[:, :3, 3], T_est[:, :3, 3]

    path_len = float(np.sum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1)))
    duration = est_rows[ie[-1], 0] - est_rows[ie[0], 0]

    _, R, t = umeyama(p_est, p_gt, with_scale=False)
    err_se3 = np.linalg.norm((R @ p_est.T).T + t - p_gt, axis=1)
    scale, _, _ = umeyama(p_est, p_gt, with_scale=True)

    # 起點對齊:T_align = T_gt[0] * inv(T_est[0]),之後完全不再修正
    T_align = T_gt[0] @ np.linalg.inv(T_est[0])
    if args.align_from:
        ref_gt, ref_est = load_tum(args.align_from[0]), load_tum(args.align_from[1])
        rg, re = associate(ref_gt[np.argsort(ref_gt[:, 0])], ref_est, args.max_dt)
        T_align = to_matrices(ref_gt[np.argsort(ref_gt[:, 0])][rg[:1]])[0] @ np.linalg.inv(
            to_matrices(ref_est[re[:1]], optical=args.est_optical)[0])
    p_anchor = (T_align[:3, :3] @ p_est.T).T + T_align[:3, 3]
    err_anchor = np.linalg.norm(p_anchor - p_gt, axis=1)

    print(f'配對筆數          : {len(ig)}(估計 {len(est_rows)} 筆,真值 {len(gt_rows)} 筆)')
    print(f'時間長度 / 路徑長 : {duration:.1f} s / {path_len:.2f} m')
    print(f'ATE RMSE (SE3)    : {np.sqrt(np.mean(err_se3 ** 2)):.4f} m(最大 {err_se3.max():.4f} m)')
    print(f'Sim3 尺度         : {scale:.4f}(雙目正確時應接近 1.0)')
    label = '對齊參考檔誤差    ' if args.align_from else '起點對齊誤差      '
    print(f'{label}: RMSE {np.sqrt(np.mean(err_anchor ** 2)):.4f} m,起點 {err_anchor[0]:.4f} m,終點 {err_anchor[-1]:.4f} m'
          + (f'(路徑長的 {100 * err_anchor[-1] / path_len:.2f}%)' if path_len > 0 else ''))

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        p_se3 = (R @ p_est.T).T + t
        fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
        ax = axes[0]
        ax.plot(p_gt[:, 0], p_gt[:, 1], 'k-', lw=2, label='ground truth')
        ax.plot(p_anchor[:, 0], p_anchor[:, 1], 'tab:orange', lw=1.2, label='VSLAM (first-pose aligned)')
        ax.plot(p_se3[:, 0], p_se3[:, 1], 'tab:blue', lw=1.2, ls='--', label='VSLAM (SE3 aligned)')
        ax.plot(p_gt[0, 0], p_gt[0, 1], 'go', label='start')
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.grid(alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        ax.set_title(args.title)
        ax = axes[1]
        tt = est_rows[ie, 0] - est_rows[ie[0], 0]
        ax.plot(tt, err_anchor, 'tab:orange', label='first-pose aligned')
        ax.plot(tt, err_se3, 'tab:blue', ls='--', label='SE3 aligned')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('position error [m]')
        ax.grid(alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        ax.set_title(f'ATE {np.sqrt(np.mean(err_se3 ** 2)):.3f} m, scale {scale:.3f}, path {path_len:.1f} m')
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f'疊圖              : {args.plot}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
