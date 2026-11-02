"""标定结果的目录与 YAML 输出（沿用 g1d_vision_data 的格式）。"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path("/home/yushu/g1d_vision_data")


def new_run_dir(kind: str, camera: str) -> Path:
    """kind: calibration / handeye"""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    d = ROOT / kind / f"{camera}_{stamp}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def write_intrinsics(path: Path, camera: str, K, D, image_size) -> Path:
    """写成 ROS camera_info 风格的 plumb_bob YAML（与旧文件一致）。"""
    K = np.asarray(K, dtype=float)
    D = np.asarray(D, dtype=float).ravel()
    while len(D) < 5:
        D = np.append(D, 0.0)
    data = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_name": camera,
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in K.ravel()]},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": [float(v) for v in D[:5]]},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": np.eye(3).ravel().tolist()},
        "projection_matrix": {"rows": 3, "cols": 4,
                              "data": [float(K[0, 0]), 0.0, float(K[0, 2]), 0.0,
                                       0.0, float(K[1, 1]), float(K[1, 2]), 0.0,
                                       0.0, 0.0, 1.0, 0.0]},
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
                    encoding="utf-8")
    return path


def write_json(path: Path, obj) -> Path:
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=1), encoding="utf-8")
    return path
