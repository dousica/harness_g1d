"""自碰撞检测：用 Pinocchio + URDF 的碰撞几何体，检查手臂是否撞到自己身上。

只用**精选碰撞对**（手臂 vs 躯干/腰部/头，以及左右臂互撞）。
不用「全对全」——全对全会碰到几对病态 mesh 导致 coal 段错误，
而精选对的组合已验证稳定（见 _PAIRS 注释）。

用法：
    sc = SelfCollision(urdf, package_dirs)
    ok, dist, why = sc.check(q14)              # 单点检查
    ok, dist, why = sc.check_path(q0, q1, 20)  # 路径采样检查
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin

from unitree_gpt6.g1d_draw.src.kinematics import ARM_JOINT_NAMES

# 碰撞几何体编号（来自 buildGeomFromUrdf 的实际顺序）
_TORSO, _WAIST, _PITCH, _HEAD = 4, 3, 2, 5
_L = {"shoulder": 6, "roll": 7, "yaw": 8, "elbow": 9,
      "wrist_roll": 10, "wrist_pitch": 11, "wrist_yaw": 12, "hand": 13}
_R = {"shoulder": 20, "roll": 21, "yaw": 22, "elbow": 23,
      "wrist_roll": 24, "wrist_pitch": 25, "wrist_yaw": 26, "hand": 27}

# 已验证可用的碰撞对（每条都单独跑过，不会崩）
_PAIRS = [
    # 左臂 vs 躯干 / 腰
    (_L["elbow"], _TORSO), (_L["elbow"], _WAIST),
    (_L["wrist_roll"], _TORSO), (_L["wrist_pitch"], _TORSO),
    (_L["wrist_yaw"], _TORSO), (_L["hand"], _TORSO), (_L["hand"], _WAIST),
    (_L["wrist_roll"], _HEAD),
    # 右臂 vs 躯干 / 腰
    (_R["elbow"], _TORSO), (_R["elbow"], _WAIST),
    (_R["wrist_roll"], _TORSO), (_R["wrist_pitch"], _TORSO),
    (_R["wrist_yaw"], _TORSO), (_R["hand"], _TORSO), (_R["hand"], _WAIST),
    (_R["wrist_roll"], _HEAD),
    # 左右臂互撞
    (_L["elbow"], _R["elbow"]), (_L["hand"], _R["hand"]),
    (_L["hand"], _R["elbow"]), (_L["elbow"], _R["hand"]),
    (_L["wrist_yaw"], _R["wrist_yaw"]),
]

SAFE_MARGIN = 0.02   # 允许的最小间距（米），小于它就拒绝动作


class SelfCollision:
    def __init__(self, urdf, package_dirs=None, margin=SAFE_MARGIN):
        self.margin = margin
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.geom = pin.buildGeomFromUrdf(
            self.model, str(urdf), pin.GeometryType.COLLISION,
            package_dirs=[str(p) for p in (package_dirs or [])])
        n = len(self.geom.geometryObjects)
        for i, j in _PAIRS:
            if i < n and j < n:
                self.geom.addCollisionPair(pin.CollisionPair(i, j))
        self.data = self.model.createData()
        self.gdata = self.geom.createData()
        self._idx = np.array([
            self.model.joints[self.model.getJointId(name)].idx_q
            for name in ARM_JOINT_NAMES])

    def _q(self, q14):
        q = pin.neutral(self.model)
        q[self._idx] = np.asarray(q14, float)
        return q

    def _distances(self, q14):
        q = self._q(q14)
        pin.updateGeometryPlacements(self.model, self.data, self.geom, self.gdata, q)
        pin.computeDistances(self.geom, self.gdata)
        return [r.min_distance for r in self.gdata.distanceResults]

    def check(self, q14):
        """返回 (是否可以, 最小间距, 说明)。"""
        try:
            ds = self._distances(q14)
        except Exception as exc:                      # noqa: BLE001
            return False, -1.0, f"碰撞检测失败：{type(exc).__name__}: {exc}"
        if not ds:
            return False, -1.0, "碰撞检测无结果"
        d = float(min(ds))
        if d <= 0:
            return False, d, f"自碰撞（间距 {d*1000:.0f} mm）"
        if d < self.margin:
            return False, d, f"间距过小（{d*1000:.0f} mm < {self.margin*1000:.0f} mm）"
        return True, d, "ok"

    def check_path(self, q_from, q_to, steps=20):
        """沿关节空间直线采样检查。返回 (是否可以, 最小间距, 说明)。"""
        a = np.asarray(q_from, float)
        b = np.asarray(q_to, float)
        worst, worst_d = True, float("inf")
        for t in np.linspace(0.0, 1.0, max(2, steps)):
            q = a + (b - a) * t
            ok, d, why = self.check(q)
            if d < worst_d:
                worst_d = d
            if not ok:
                return False, d, f"路径 t={t:.2f} 处{why}"
        return worst, worst_d, "ok"
