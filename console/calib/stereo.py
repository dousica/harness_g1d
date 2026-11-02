"""双目标定：同步图对 → 各自内参 → 外参。

一次采集同时产出三样东西：
    head_left.yaml / head_right.yaml / head_stereo.yaml
"""
from __future__ import annotations

import cv2
import numpy as np

from board import BOARD, BoardSpec, board_object_points


class StereoSession:
    def __init__(self, left="head_left", right="head_right", spec: BoardSpec = BOARD):
        self.left, self.right, self.spec = left, right, spec
        self.obj: list[np.ndarray] = []
        self.imgL: list[np.ndarray] = []
        self.imgR: list[np.ndarray] = []
        self.names: list[str] = []
        self.image_size = None

    def add(self, imgL, cL, idL, imgR, cR, idR, name="") -> tuple[bool, str]:
        if cL is None or cR is None or idL is None or idR is None:
            return False, "有一只眼睛没检测到板子"
        common = np.intersect1d(np.asarray(idL).ravel(), np.asarray(idR).ravel())
        if len(common) < 12:
            return False, f"共同角点太少（{len(common)}）"
        idxL = {int(v): i for i, v in enumerate(np.asarray(idL).ravel())}
        idxR = {int(v): i for i, v in enumerate(np.asarray(idR).ravel())}
        pL = np.asarray(cL).reshape(-1, 2)[[idxL[int(c)] for c in common]]
        pR = np.asarray(cR).reshape(-1, 2)[[idxR[int(c)] for c in common]]
        obj = board_object_points(common, self.spec)
        self.obj.append(obj.astype(np.float32))
        self.imgL.append(pL.astype(np.float32))
        self.imgR.append(pR.astype(np.float32))
        self.names.append(name)
        self.image_size = (int(imgL.shape[1]), int(imgL.shape[0]))
        return True, "ok"

    def calibrate(self):
        if len(self.obj) < 6:
            raise ValueError(f"同步图对太少：{len(self.obj)}（至少 6 对，建议 15+）")
        size = self.image_size
        # 实测教训：CALIB_RATIONAL_MODEL 在 stereoCalibrate 里会让解算崩掉
        # （单目 RMS 0.24 正常，双目 RMS 7722、基线 7.6mm）。固定用 5 参数模型。
        flags = cv2.CALIB_ZERO_TANGENT_DIST
        rmsL, KL, DL, _, _ = cv2.calibrateCamera(self.obj, self.imgL, size, None, None, flags=flags)
        rmsR, KR, DR, _, _ = cv2.calibrateCamera(self.obj, self.imgR, size, None, None, flags=flags)
        rmsS, KL, DL, KR, DR, R, T, E, F = cv2.stereoCalibrate(
            self.obj, self.imgL, self.imgR, KL, DL, KR, DR, size,
            flags=cv2.CALIB_FIX_INTRINSIC)
        baseline = float(np.linalg.norm(T))
        # 自检：效果不对就拒绝出结果，而不是写一个坏文件
        if not np.isfinite(rmsS) or rmsS > 2.0:
            raise ValueError(f"双目 RMS {rmsS:.1f} px 过大（>2.0），标定失败")
        if not (0.02 <= baseline <= 0.30):
            raise ValueError(f"基线 {baseline*1000:.1f} mm 不合理（应在 20~300 mm）")
        return {
            "rms_left": float(rmsL), "rms_right": float(rmsR), "rms_stereo": float(rmsS),
            "baseline_m": baseline,
            "K_left": KL.tolist(), "D_left": np.asarray(DL).ravel().tolist(),
            "K_right": KR.tolist(), "D_right": np.asarray(DR).ravel().tolist(),
            "R": R.tolist(), "T": np.asarray(T).ravel().tolist(),
            "image_size": list(size), "pairs": len(self.obj),
        }

    def add_from_data(self, obj, imgL, imgR, name=""):
        self.obj.append(np.asarray(obj, dtype=np.float32))
        self.imgL.append(np.asarray(imgL, dtype=np.float32))
        self.imgR.append(np.asarray(imgR, dtype=np.float32))
        self.names.append(name)

    def __len__(self):
        return len(self.obj)
