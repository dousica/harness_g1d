#!/usr/bin/env python3
"""Publish the calibrated fixed head-camera transforms on ``/tf_static``.

The left-camera transform is read from the eye-to-hand result.  The right-camera
transform is derived from the stereo result, so it does not need a second hand-eye
calibration:

    T_torso_right = T_torso_left @ inverse(T_left_right)

Both calibration files describe raw optical camera frames.  This node deliberately
does not publish a transform for a rectified image frame.
"""

from __future__ import annotations

import math
import os
import hashlib
from pathlib import Path
from typing import Iterable

import yaml


def _matrix4(value: object, name: str) -> list[list[float]]:
    rows = [[float(x) for x in row] for row in value]  # type: ignore[union-attr]
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise ValueError(f"{name} must be a 4x4 matrix")
    if any(not math.isfinite(x) for row in rows for x in row):
        raise ValueError(f"{name} contains a non-finite value")
    return rows


def _matrix3(value: object, name: str) -> list[list[float]]:
    rows = [[float(x) for x in row] for row in value]  # type: ignore[union-attr]
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError(f"{name} must be a 3x3 matrix")
    if any(not math.isfinite(x) for row in rows for x in row):
        raise ValueError(f"{name} contains a non-finite value")
    return rows


def _vector3(value: object, name: str) -> list[float]:
    values = list(value)  # type: ignore[arg-type]
    if len(values) == 3 and all(isinstance(x, (list, tuple)) for x in values):
        if any(len(x) != 1 for x in values):  # type: ignore[arg-type]
            raise ValueError(f"{name} must contain three values")
        values = [x[0] for x in values]  # type: ignore[index]
    values = [float(x) for x in values]
    if len(values) != 3 or any(not math.isfinite(x) for x in values):
        raise ValueError(f"{name} must contain three finite values")
    return values


def _from_rt(rotation: list[list[float]], translation: Iterable[float]) -> list[list[float]]:
    t = list(translation)
    return [rotation[0] + [t[0]], rotation[1] + [t[1]], rotation[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def _rigid_inverse(t: list[list[float]]) -> list[list[float]]:
    r = [row[:3] for row in t[:3]]
    p = [t[i][3] for i in range(3)]
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    neg = [-sum(rt[i][k] * p[k] for k in range(3)) for i in range(3)]
    return _from_rt(rt, neg)


def _quat_xyzw(rotation: list[list[float]]) -> tuple[float, float, float, float]:
    """Convert a proper rotation matrix to a ROS x,y,z,w quaternion."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2][1] - rotation[1][2]) / s
        y = (rotation[0][2] - rotation[2][0]) / s
        z = (rotation[1][0] - rotation[0][1]) / s
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        s = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        w = (rotation[2][1] - rotation[1][2]) / s
        x = 0.25 * s
        y = (rotation[0][1] + rotation[1][0]) / s
        z = (rotation[0][2] + rotation[2][0]) / s
    elif rotation[1][1] > rotation[2][2]:
        s = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        w = (rotation[0][2] - rotation[2][0]) / s
        x = (rotation[0][1] + rotation[1][0]) / s
        y = 0.25 * s
        z = (rotation[1][2] + rotation[2][1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
        w = (rotation[1][0] - rotation[0][1]) / s
        x = (rotation[0][2] + rotation[2][0]) / s
        y = (rotation[1][2] + rotation[2][1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("rotation matrix produced an invalid quaternion")
    return x / norm, y / norm, z / norm, w / norm


def load_transforms(handeye_path: str, stereo_path: str) -> tuple[list[list[float]], list[list[float]]]:
    handeye_file = Path(handeye_path).expanduser()
    if handeye_file.is_dir():
        candidates = sorted(
            handeye_file.rglob("handeye_result_*.yaml"),
            key=lambda item: item.stat().st_mtime,
            reverse=True,
        )
        if not candidates:
            raise FileNotFoundError(
                f"no handeye_result_*.yaml under {handeye_file}"
            )
        handeye_file = candidates[0]
    handeye_text = handeye_file.read_text(encoding="utf-8")
    stereo_text = Path(stereo_path).expanduser().read_text(encoding="utf-8")
    handeye = yaml.safe_load(handeye_text)
    stereo = yaml.safe_load(stereo_text)
    if handeye.get("base_frame") != "torso_link":
        raise ValueError("hand-eye result base_frame must be torso_link")
    if handeye.get("camera_frame") != "head_left_camera_optical_frame":
        raise ValueError("hand-eye result must be for head_left_camera_optical_frame")
    if handeye.get("units") != "meters":
        raise ValueError("hand-eye result units must be meters")
    expected_hash = handeye.get("intrinsics_sha256")
    actual_hash = hashlib.sha256(stereo_text.encode("utf-8")).hexdigest()
    if expected_hash != actual_hash:
        raise ValueError(
            "stereo calibration does not match the hand-eye session intrinsics "
            f"(expected SHA256 {expected_hash}, got {actual_hash})"
        )
    if stereo.get("transform_convention") != (
        "X_right = R_left_to_right @ X_left + T_left_to_right_m"
    ):
        raise ValueError("stereo result has an unexpected transform convention")
    torso_left = _matrix4(handeye["T_torso_camera"], "T_torso_camera")
    left_right = _from_rt(
        _matrix3(stereo["R_left_to_right"], "R_left_to_right"),
        _vector3(stereo["T_left_to_right_m"], "T_left_to_right_m"),
    )
    torso_right = _matmul(torso_left, _rigid_inverse(left_right))
    return torso_left, torso_right


def main() -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    class HeadCameraTFPublisher(Node):
        def __init__(self) -> None:
            super().__init__("lzh_head_camera_tf_publisher")
            home = os.environ.get("HOME", "/home/yushu")
            self.declare_parameter(
                "handeye_result",
                f"{home}/g1d_vision_data/handeye",
            )
            self.declare_parameter(
                "stereo_result",
                f"{home}/.ros/g1d_camera_calibration/head_stereo.yaml",
            )
            self.declare_parameter("parent_frame", "torso_link")
            self.declare_parameter("left_frame", "head_left_camera_optical_frame")
            self.declare_parameter("right_frame", "head_right_camera_optical_frame")

            handeye_path = str(self.get_parameter("handeye_result").value)
            stereo_path = str(self.get_parameter("stereo_result").value)
            parent = str(self.get_parameter("parent_frame").value)
            left_frame = str(self.get_parameter("left_frame").value)
            right_frame = str(self.get_parameter("right_frame").value)
            if parent != "torso_link" or left_frame != "head_left_camera_optical_frame":
                raise ValueError("This calibrated result is fixed to the G1-D torso/left optical frames")

            torso_left, torso_right = load_transforms(handeye_path, stereo_path)
            self.broadcaster = StaticTransformBroadcaster(self)
            messages = [
                self._message(parent, left_frame, torso_left),
                self._message(parent, right_frame, torso_right),
            ]
            self.broadcaster.sendTransform(messages)
            self.get_logger().info(f"Published static TF: {parent} -> {left_frame}")
            self.get_logger().info(f"Published static TF: {parent} -> {right_frame} (derived from stereo)")
            self.get_logger().info(
                "Camera TF calibration files: handeye=%s stereo=%s" % (handeye_path, stereo_path)
            )
            for name, transform in ((left_frame, torso_left), (right_frame, torso_right)):
                q = _quat_xyzw([row[:3] for row in transform[:3]])
                self.get_logger().info(
                    "%s translation=[%.6f, %.6f, %.6f] quaternion_xyzw=[%.6f, %.6f, %.6f, %.6f]"
                    % (name, transform[0][3], transform[1][3], transform[2][3], *q)
                )

        @staticmethod
        def _message(parent: str, child: str, transform: list[list[float]]) -> TransformStamped:
            msg = TransformStamped()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.translation.x = transform[0][3]
            msg.transform.translation.y = transform[1][3]
            msg.transform.translation.z = transform[2][3]
            q = _quat_xyzw([row[:3] for row in transform[:3]])
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
            return msg

    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain != "42":
        raise RuntimeError(f"ROS_DOMAIN_ID must be 42; got {domain!r}")
    for variable in (
        "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
        "PYTHONPATH", "LD_LIBRARY_PATH"
    ):
        for entry in os.environ.get(variable, "").split(os.pathsep):
            if any(marker in entry for marker in (
                "g1d_ros2_ws", "g1d_ros2_yqqh", "unitree_ros_official"
            )):
                raise RuntimeError(f"{variable} contains forbidden overlay path: {entry}")

    rclpy.init()
    node = None
    try:
        node = HeadCameraTFPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        from rclpy.executors import ExternalShutdownException

        if not isinstance(exc, ExternalShutdownException):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
