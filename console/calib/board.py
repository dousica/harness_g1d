"""ChArUco 标定板定义（沿用 g1d_vision_data 里已用的参数）。

来源：/home/yushu/g1d_vision_data/handeye/*/samples.json
    "board": {"squares": [12, 9], "square_length_m": 0.015,
              "marker_length_m": 0.01125, "dictionary": "DICT_5X5_250"}
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class BoardSpec:
    squares_x: int = 12
    squares_y: int = 9
    square_length_m: float = 0.015
    marker_length_m: float = 0.01125
    dictionary: str = "DICT_5X5_250"

    @property
    def total_corners(self) -> int:
        """ChArUco 有效内角点数 = (sx-1) * (sy-1)。"""
        return (self.squares_x - 1) * (self.squares_y - 1)


BOARD = BoardSpec()


def make_dictionary(name: str = BOARD.dictionary):
    dict_id = getattr(cv2.aruco, name, None)
    if dict_id is None:
        raise ValueError(f"未知的 ArUco 字典：{name}")
    return cv2.aruco.getPredefinedDictionary(dict_id)


def make_board(spec: BoardSpec = BOARD):
    return cv2.aruco.CharucoBoard(
        (spec.squares_x, spec.squares_y),
        spec.square_length_m,
        spec.marker_length_m,
        make_dictionary(spec.dictionary),
    )


def make_detector(spec: BoardSpec = BOARD):
    """OpenCV >= 4.7 的 CharucoDetector API。"""
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(make_board(spec))
    detector.setDetectorParameters(params)
    return detector


def detect(image_bgr, detector):
    """返回 (charuco_corners Nx1x2, charuco_ids Nx1)。检测不到返回 (None, None)。"""
    if image_bgr is None:
        return None, None
    gray = image_bgr if image_bgr.ndim == 2 else cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _marker_corners, _marker_ids = detector.detectBoard(gray)
    if ids is None or len(ids) == 0:
        return None, None
    return corners, ids


def board_object_points(ids, spec: BoardSpec = BOARD):
    """按检测到的角点 id 取出对应的 3D 坐标（米）。"""
    board = make_board(spec)
    all_points = board.getChessboardCorners()
    return np.array([all_points[int(i)] for i in np.asarray(ids).ravel()], dtype=np.float64)
