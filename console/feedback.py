"""电机反馈采样：tau_est / temperature / vol。

官方 MotorState 每个电机都带这些字段，但 g1d_draw 的后端只取了 q 和 dq。
这里单独订阅 rt/lowstate，把安全相关的信号捞出来，供：
  * 夹爪力控（判断"夹住了没有"）
  * 碰撞检测（外力矩残差）
  * 温度/电压监控
"""
from __future__ import annotations

import threading
import time

import numpy as np

N_MOTORS = 35


class MotorFeedback:
    """只读订阅 rt/lowstate，不发送任何指令。"""

    def __init__(self):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        self.lock = threading.Lock()
        self.q = np.zeros(N_MOTORS)
        self.dq = np.zeros(N_MOTORS)
        self.tau = np.zeros(N_MOTORS)
        self.temp = np.zeros(N_MOTORS)
        self.vol = np.zeros(N_MOTORS)
        self.stamp = 0.0
        self.count = 0
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_msg, 10)

    def _on_msg(self, msg):
        n = min(N_MOTORS, len(msg.motor_state))
        with self.lock:
            for i in range(n):
                st = msg.motor_state[i]
                self.q[i] = st.q
                self.dq[i] = st.dq
                self.tau[i] = st.tau_est
                self.temp[i] = st.temperature[0]
                self.vol[i] = st.vol
            self.stamp = time.monotonic()
            self.count += 1

    # --- 便捷读数 ---------------------------------------------------------
    def grip_tau(self):
        """左右夹爪的估计力矩（绝对值）。"""
        with self.lock:
            return np.abs(self.tau[[31, 33]]).copy()

    def grip_q(self):
        with self.lock:
            return self.q[[31, 33]].copy()

    def arm_tau(self, slots):
        with self.lock:
            return self.tau[list(slots)].copy()

    def fresh(self, timeout=0.5):
        with self.lock:
            return (time.monotonic() - self.stamp) < timeout

    def snapshot(self):
        with self.lock:
            return {
                "tau": self.tau.copy(),
                "temp": self.temp.copy(),
                "vol": self.vol.copy(),
                "q": self.q.copy(),
                "fresh": (time.monotonic() - self.stamp) < 0.5,
            }
