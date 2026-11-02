"""标定图像质量评分：判断一张图"能不能用"。

四个维度（都是实践证明有效的）：
  * 角点数      —— 太少解不稳定
  * 覆盖率      —— 板子只占画面一小块时，畸变参数学不出来
  * 倾斜角      —— 正对镜头和极斜都贡献小
  * 清晰度      —— 模糊帧会让角点亚像素精度崩掉
"""
from __future__ import annotations

import cv2
import numpy as np

MIN_CORNERS_RATIO = 0.60    # 至少检测到 60% 的角点（整块板在画面里）
MIN_AREA_RATIO = 0.03       # 板子凸包至少占画面 3%（广角镜头下 5% 仍偏严）
MIN_SHARPNESS = 100.0       # Laplacian 方差
MIN_TILT_DEG = 12.0         # 与镜头夹角至少 12°
MAX_TILT_DEG = 75.0


def metrics(image_bgr, corners, ids, total_corners: int) -> dict:
    gray = image_bgr if image_bgr.ndim == 2 else cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    out = {
        "ok": False,
        "corner_count": 0,
        "corner_ratio": 0.0,
        "area_ratio": 0.0,
        "sharpness": 0.0,
        "tilt_deg": None,
        "reasons": [],
    }
    # 清晰度与有没有板子无关，先算出来（方便调焦）
    out["sharpness"] = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    if corners is None or ids is None or len(ids) == 0:
        out["reasons"].append("没有检测到标定板")
        return out

    pts = np.asarray(corners).reshape(-1, 2)
    out["corner_count"] = int(len(pts))
    out["corner_ratio"] = len(pts) / float(max(1, total_corners))
    hull = cv2.convexHull(pts.reshape(-1, 1, 2))
    out["area_ratio"] = float(cv2.contourArea(hull)) / float(w * h)

    if out["corner_ratio"] < MIN_CORNERS_RATIO:
        out["reasons"].append(f"角点太少 {len(pts)}/{total_corners}")
    if out["area_ratio"] < MIN_AREA_RATIO:
        out["reasons"].append(f"覆盖面积 {out['area_ratio']*100:.0f}% < {MIN_AREA_RATIO*100:.0f}%")
    if out["sharpness"] < MIN_SHARPNESS:
        out["reasons"].append(f"图像模糊（清晰度 {out['sharpness']:.0f}）")

    out["ok"] = not out["reasons"]
    return out


def pose_diversity(new_rvec, existing_rvecs, min_deg=8.0):
    """与已保存位姿的最小旋转差（度）。太小说明这张图没带来新信息。"""
    if not existing_rvecs:
        return float("inf")
    R_new, _ = cv2.Rodrigues(new_rvec)
    worst = float("inf")
    for rv in existing_rvecs:
        R_old, _ = cv2.Rodrigues(rv)
        cos = (np.trace(R_new @ R_old.T) - 1.0) / 2.0
        ang = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))
        worst = min(worst, ang)
    return worst
