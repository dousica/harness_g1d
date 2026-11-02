"""手眼标定（eye-to-hand）：Park-Martin 旋转对数对齐 + 最小二乘平移。

沿用 g1d_ros2_ws/g1d_handeye_gui.py 的算法（纯 NumPy）：
    G_i Y = X C_i
    G = torso_T_tcp        （机械臂 FK）
    C = camera_T_board     （ChArUco）
    X = torso_T_camera     （要求解的固定变换）
参考：Park & Martin, IEEE TRA 10(5), 717-721, 1994, doi:10.1109/70.326576
"""
from __future__ import annotations

import numpy as np


def inverse(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def rotation_log(R):
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    skew = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if theta < 1e-7:
        return skew / 2
    if np.pi - theta < 1e-5:
        raise ValueError("相对旋转接近 180°，此样本无法使用")
    return theta * skew / (2 * np.sin(theta))


def mean_pose(poses):
    out = np.eye(4)
    U, _, Vt = np.linalg.svd(sum(T[:3, :3] for T in poses))
    out[:3, :3] = U @ np.diag([1.0, 1.0, np.linalg.det(U @ Vt)]) @ Vt
    out[:3, 3] = np.mean([T[:3, 3] for T in poses], axis=0)
    return out


def pose_delta(A, B):
    D = inverse(A) @ B
    ang = np.rad2deg(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
    return float(np.linalg.norm(A[:3, 3] - B[:3, 3])), float(ang)


def solve_eye_to_hand(G, C, min_pairs=12):
    """G: list of torso_T_tcp；C: list of camera_T_board。返回 (X, Y, 信息)。"""
    if len(G) < 3:
        raise ValueError(f"样本太少：{len(G)}（至少 3 个，建议 12+）")
    pairs, H = [], np.zeros((3, 3))
    for i in range(len(G)):
        for j in range(i):
            A, B = G[i] @ inverse(G[j]), C[i] @ inverse(C[j])
            ang_a = np.rad2deg(np.arccos(np.clip((np.trace(A[:3, :3]) - 1) / 2, -1, 1)))
            ang_b = np.rad2deg(np.arccos(np.clip((np.trace(B[:3, :3]) - 1) / 2, -1, 1)))
            if not (3 < ang_a < 170 and 3 < ang_b < 170):
                continue
            H += np.outer(rotation_log(A[:3, :3]), rotation_log(B[:3, :3]))
            pairs.append((A, B))
    if len(pairs) < 3:
        raise ValueError("有效旋转对太少：请让腕部绕两个不平行的轴转动")
    U, singular, Vt = np.linalg.svd(H)
    if singular[0] < 1e-8 or singular[1] / singular[0] < 0.02:
        raise ValueError("旋转多样性不足：至少绕两个不平行的轴转")
    R = U @ np.diag([1.0, 1.0, np.linalg.det(U @ Vt)]) @ Vt
    M = np.vstack([A[:3, :3] - np.eye(3) for A, _ in pairs])
    rhs = np.concatenate([R @ B[:3, 3] - A[:3, 3] for A, B in pairs])
    t, _, rank, _ = np.linalg.lstsq(M, rhs, rcond=None)
    if rank < 3:
        raise ValueError("平移不可观测：旋转还不够")
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, t
    Y = mean_pose([inverse(g) @ X @ c for g, c in zip(G, C)])
    return X, Y, {"pairs": len(pairs), "singular": singular.tolist()}


def residuals(G, C, X, Y):
    rows = []
    for i, (g, c) in enumerate(zip(G, C)):
        cand = inverse(g) @ X @ c
        m, deg = pose_delta(Y, cand)
        rows.append({"index": i, "translation_mm": m * 1000, "rotation_deg": deg})
    if not rows:
        return {"count": 0, "per_sample": []}
    return {
        "count": len(rows),
        "translation_rms_mm": float(np.sqrt(np.mean([r["translation_mm"] ** 2 for r in rows]))),
        "rotation_rms_deg": float(np.sqrt(np.mean([r["rotation_deg"] ** 2 for r in rows]))),
        "per_sample": rows,
    }


class TcpKinematics:
    """用 Pinocchio 算 torso_T_tcp（替代旧脚本依赖的 ROS TF）。"""

    def __init__(self, urdf, package_dirs=None,
                 base_frame="torso_link", tcp_frames=("left_dex1_tcp_link",
                                                      "right_dex1_tcp_link")):
        import pinocchio as pin
        from unitree_gpt6.g1d_draw.src.kinematics import ARM_JOINT_NAMES

        self._pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.base_id = self.model.getFrameId(base_frame)
        self.tcp_ids = {f: self.model.getFrameId(f) for f in tcp_frames}
        self.idx = np.array([self.model.joints[self.model.getJointId(n)].idx_q
                             for n in ARM_JOINT_NAMES])

    def torso_T_tcp(self, q14, tcp="left_dex1_tcp_link"):
        pin = self._pin
        q = pin.neutral(self.model)
        q[self.idx] = np.asarray(q14, float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        T_base = self.data.oMf[self.base_id]
        T_tcp = self.data.oMf[self.tcp_ids[tcp]]
        return inverse(T_base.homogeneous) @ T_tcp.homogeneous
