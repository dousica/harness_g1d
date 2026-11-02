"""单目标定：采集样本 → cv2.calibrateCamera → YAML。"""
from __future__ import annotations

import cv2
import numpy as np

from board import BOARD, BoardSpec, board_object_points


class IntrinsicSession:
    def __init__(self, camera: str, spec: BoardSpec = BOARD):
        self.camera = camera
        self.spec = spec
        self.obj_points: list[np.ndarray] = []
        self.img_points: list[np.ndarray] = []
        self.rvecs: list[np.ndarray] = []
        self.image_names: list[str] = []
        self.image_size = None
        self._K = None
        self._D = None

    # --- 采集 -------------------------------------------------------------
    def add(self, image_bgr, corners, ids, name: str = "") -> tuple[bool, str]:
        """加入一张图。返回 (是否接受, 原因)。"""
        if corners is None or ids is None or len(ids) < 8:
            return False, "角点不足"
        obj = board_object_points(ids, self.spec)
        img = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        h, w = image_bgr.shape[:2]
        if self.image_size is None:
            self.image_size = (w, h)
        elif self.image_size != (w, h):
            return False, f"尺寸不一致（{w}x{h} vs {self.image_size[0]}x{self.image_size[1]}）"

        # 用 solvePnP 估个粗略位姿，用于多样性判断
        if self._K is None:
            k = np.eye(3, dtype=np.float64)
            k[0, 0] = k[1, 1] = 0.9 * max(w, h)
            k[0, 2], k[1, 2] = w / 2.0, h / 2.0
            self._K = k
        ok, rvec, tvec = cv2.solvePnP(obj, img, self._K, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return False, "位姿解算失败"

        from quality import pose_diversity
        div = pose_diversity(rvec, self.rvecs)
        if div < 6.0:
            return False, f"与已有图太相似（差 {div:.1f}°，需 >6°）"

        # calibrateCamera 要求 Point3f / Point2f（float32）
        self.obj_points.append(np.asarray(obj, dtype=np.float32))
        self.img_points.append(np.asarray(img, dtype=np.float32).reshape(-1, 1, 2))
        self.rvecs.append(rvec)
        self.image_names.append(name)
        return True, "ok"

    # --- 解算 -------------------------------------------------------------
    def calibrate(self):
        if len(self.obj_points) < 8:
            raise ValueError(f"样本太少：{len(self.obj_points)} 张（至少 8 张，建议 20+）")
        flags = cv2.CALIB_ZERO_TANGENT_DIST
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, self.image_size, None, None, flags=flags)
        per_view = []
        for i, (obj, img, rv, tv) in enumerate(zip(self.obj_points, self.img_points, rvecs, tvecs)):
            proj, _ = cv2.projectPoints(obj, rv, tv, K, D)
            err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
            per_view.append({"index": i, "name": self.image_names[i], "rms_px": round(err, 3)})
        return {"rms": float(rms), "K": K.tolist(), "D": np.asarray(D).ravel().tolist(),
                "image_size": list(self.image_size), "per_view": per_view,
                "samples": len(self.obj_points)}

    def add_from_data(self, obj_points, img_points, name=""):
        """从磁盘恢复一个样本（跳过质量检查）。"""
        self.obj_points.append(np.asarray(obj_points, dtype=np.float32))
        self.img_points.append(np.asarray(img_points, dtype=np.float32).reshape(-1, 1, 2))
        self.image_names.append(name)
        if len(self.img_points) == 1:
            # 用第一张图推一个粗略位姿，供后续多样性判断
            pass

    def __len__(self):
        return len(self.obj_points)
