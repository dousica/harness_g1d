"""控制环数据记录 + 异常检测。

出问题时最缺的是「每一拍到底发了什么、机器人回了什么」。
这里把控制环的数据按 20 Hz 写成 JSONL，出事可以直接翻文件定位。

文件：logs/run_YYYYmmdd_HHMMSS.jsonl
每行一个 JSON：时间、关节 q/cmd、夹爪 cmd/q/tau、力矩、温度、状态。
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np


class Recorder:
    def __init__(self, directory="logs", every=5):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path = self.dir / time.strftime("run_%Y%m%d_%H%M%S.jsonl")
        self.fh = open(self.path, "a", encoding="utf-8", buffering=1)
        self.every = every
        self.tick = 0

    def log(self, **fields):
        self.tick += 1
        if self.tick % self.every:
            return
        row = {"t": round(time.time(), 3)}
        for k, v in fields.items():
            if isinstance(v, np.ndarray):
                row[k] = np.round(v.astype(float), 4).tolist()
            elif isinstance(v, (np.floating, np.integer)):
                row[k] = float(v)
            else:
                row[k] = v
        self.fh.write(json.dumps(row, ensure_ascii=False) + "\n")

    def close(self):
        try:
            self.fh.close()
        except Exception:
            pass


class Anomaly:
    @staticmethod
    def check(tau_arm, temp, volt, grip_cmd, grip_q,
              joint_tau_max=45.0, temp_max=70.0, volt_min=40.0):
        """返回 (是否异常, 说明, 级别)。级别：fault=立即急停，warn=仅记录。"""
        if len(tau_arm):
            worst = float(np.max(np.abs(tau_arm)))
            if worst > joint_tau_max:
                return True, f"关节力矩 {worst:.1f} 超过上限 {joint_tau_max:.0f}", "fault"
        if len(temp):
            tmax = float(np.max(temp))
            if tmax > temp_max:
                return True, f"电机温度 {tmax:.0f}°C 超过上限 {temp_max:.0f}°C", "fault"
        if len(volt):
            alive = volt[volt > 1.0]
            if len(alive) and float(np.min(alive)) < volt_min:
                return True, f"电压偏低 {float(np.min(alive)):.1f}V", "warn"
        err = abs(float(grip_cmd) - float(grip_q))
        if err > 0.006:
            return True, f"夹爪未跟随（指令 {grip_cmd:.4f} / 实测 {grip_q:.4f}）", "warn"
        return False, "", "ok"
