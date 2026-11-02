"""G1-D 控制台核心：手臂控制、相机、示教库、GPT-6 Astra 任务。

安全设计（都是踩过坑后加的）
---------------------------
* 被动启动：只连接、只读；接管必须由人点按钮触发。
* 防饱和：指令位置每拍被夹在「实测 ± LEAD」内，卡住时指令自动停住，
  不会累积势能（这是之前撞桌后抽搐的根因）。
* 堵转检测：差值持续超限 1.2 s 判定被挡住，立即停止下发并交回机器人。
* 软限位：每个关节内缩 0.08 rad，绝不顶到机械硬限位。
* 退出/故障：SelectMode('ai') 把控制权还给机器人，避免手臂失力下坠。
"""
from __future__ import annotations

import base64
import json
import os
import threading
import time
import traceback
import urllib.request
from pathlib import Path

import numpy as np

from unitree_gpt6.g1d_draw.src.config import load_config
from unitree_gpt6.g1d_draw.src.g1 import G1DDSBackend
from unitree_gpt6.g1d_draw.src.kinematics import PinocchioKinematics, ARM_JOINT_NAMES

from logger import Anomaly, Recorder


def _urdf_effort_limits(urdf_path):
    """从 URDF 读每个手臂关节的 effort 上限（官方机械限制）。"""
    import re
    from pathlib import Path
    try:
        src = Path(urdf_path).read_text(encoding="utf-8")
    except Exception:
        return None
    out = []
    for name in ARM_JOINT_NAMES:
        m = re.search(rf'<joint name="{name}"[^>]*>(.*?)</joint>', src, re.S)
        lim = re.search(r'effort="([0-9.]+)"', m.group(1)) if m else None
        out.append(float(lim.group(1)) if lim else 0.0)
    return np.array(out) if all(v > 0 for v in out) else None

ROOT = Path(__file__).resolve().parent
LIB = ROOT / "waypoints.json"
HOME_FILE = ROOT / "home.json"
DANGLE_FILE = ROOT / "dangle.json"
ZERO_POSE = np.zeros(14)          # 官方零位：14 个关节全为 0

GRIP_SLOTS = (31, 33)
# Dex1 夹爪是 **prismatic（移动关节）**，URDF: lower=-0.02 upper=0.0245 (米)
# 行程 45 mm，速度上限 0.2 m/s。留 2 mm 余量，绝不允许顶到硬限位。
GRIP_OPEN, GRIP_CLOSE = 3.3799, -0.0501   # 实测行程（可被 gripper_range.json 覆盖）

TICK = 0.01
MAX_STEP = 0.15
V_MAX, A_MAX = 0.25, 0.50   # 关节速度上限 rad/s、加速度上限 rad/s^2
GRIP_RATE = 0.80            # 夹爪开合速度（归一化刻度/秒，全程约 4 秒）
GRIP_TAU_MAX = 2.0          # 夹爪力矩上限，超过就双向冻结（力控）
GRIP_TAU_GRASP = 0.60       # 判定"夹住东西"的力矩阈值（空夹到底约 0.39，所以设 0.6）
EFFORT_SAFETY = 0.80        # 机械 effort 上限的使用比例（超过立即急停）
HARD_TAU_FACTOR = 0.95      # 绝对硬顶：effort 的 95%，任何情况都不允许
TEMP_MAX = 70.0             # 电机温度上限（摄氏度）
VOLT_MIN = 44.0             # 电压下限（标称 50V；低于此值提前告警，避免运行中欠压关机）
LEAD = 0.12            # 指令最多领先实测多少（rad）
LIMIT_MARGIN = 0.08    # 关节软限位内缩（rad）
STALL_SEC = 1.2        # 差值持续超限多久判定堵转
STALL_FRAC = 0.9       # 用 LEAD 的该比例作为堵转判据
OSC_TAU = 8.0          # 力矩在 0.2 秒内起伏超过这么多，判定为关节振荡（通常是控制器打架）
HB_TIMEOUT = 8.0       # 浏览器心跳多少秒没来就自动急停（页面切走也会触发，这是有意的）

CAM_HOST = os.environ.get("G1D_CAM_HOST", "192.168.123.164")
# 注意：机器人配置里 left_wrist=55556 / right_wrist=55557，
# 但实际接线是反的（已由现场确认），所以这里对调。
CAM_SOURCES = {
    "head": (f"tcp://{CAM_HOST}:55555", "stereo"),
    "lwrist": (f"tcp://{CAM_HOST}:55557", "single"),
    "rwrist": (f"tcp://{CAM_HOST}:55556", "single"),
}

API_BASE = os.environ.get("ASTRA_BASE_URL", "https://www.inroi.shop/v1")
API_KEY = os.environ.get("ASTRA_API_KEY", "")
MODEL = os.environ.get("ASTRA_MODEL", "gpt-6-astra")


# ------------------------------------------------------------------ mode switch
def ensure_debug_mode():
    """rt/lowcmd 只在 debug 模式被接受；那里机器人不吃它。"""
    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )
        msc = MotionSwitcherClient()
        msc.SetTimeout(1.0)
        msc.Init()
        status, result = msc.CheckMode()
        name = (result or {}).get("name") or ""
        if name:
            msc.ReleaseMode()
            time.sleep(1.0)
            status, result = msc.CheckMode()
            name = (result or {}).get("name") or ""
        return ("debug" if not name else f"still:{name}"), None
    except Exception as exc:                      # noqa: BLE001
        return "unknown", f"{type(exc).__name__}: {exc}"


def hand_back_to_robot():
    """把控制权交回机器人自己的控制器（ai 模式）。"""
    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )
        msc = MotionSwitcherClient()
        msc.SetTimeout(1.0)
        msc.Init()
        msc.SelectMode("ai")
        print("[arm] 已交回机器人控制器（ai 模式）", flush=True)
    except Exception as exc:                      # noqa: BLE001
        print(f"[arm] 交回控制器失败：{exc}", flush=True)


# --------------------------------------------------------------------------- arm
class Arm:
    def __init__(self, config_path):
        self.cfg = load_config(config_path)
        self.kin = PinocchioKinematics(self.cfg)
        self.backend = None
        self.engaged = False
        self.mode = "idle"
        self.fault = None
        self.grip_val = GRIP_OPEN
        self.estopped = False
        self.estop_reason = None
        self.last_heartbeat = time.monotonic()
        self.hb_enabled = False       # 心跳看门狗：默认关闭，界面上可开
        self.connect_state = {"state": "idle", "attempt": 0, "total": 4, "error": None}
        self._engage_thread = None
        self.lock = threading.Lock()
        self.alive = True
        self.collision = None
        try:
            from collision import SelfCollision
            mcfg = self.cfg["model"] if isinstance(self.cfg, dict) else self.cfg.model
            dirs = list(mcfg.get("package_dirs") or []) or ["/home/yushu/g1d_ros2_lzh/src"]
            self.collision = SelfCollision(mcfg["urdf"], dirs)
            print(f"[arm] 自碰撞检测已启用（{len(self.collision.geom.collisionPairs)} 对，"
                  f"安全间距 {self.collision.margin*1000:.0f} mm）", flush=True)
        except Exception as exc:
            print(f"[arm] 自碰撞检测不可用：{exc} —— 所有运动将被拒绝", flush=True)
        self.rec = None
        self._connect()
        s = None
        for _ in range(30):
            try:
                s = self.backend.read()
                break
            except Exception:
                time.sleep(0.2)
        if s is None:
            raise RuntimeError("DDS 已连接但读不到状态")
        q = np.array(s["q14"], float)
        dang = None
        if DANGLE_FILE.exists():
            try:
                dang = np.array(json.loads(DANGLE_FILE.read_text(encoding="utf-8"))["q"], float)
            except Exception:
                dang = None
        if dang is not None and dang.shape == q.shape:
            self.dangle = dang
        else:
            self.dangle = q.copy()
            try:
                DANGLE_FILE.write_text(json.dumps({"q": self.dangle.tolist()}, indent=1),
                                       encoding="utf-8")
                print("[arm] 已记录自然下垂位 dangle.json", flush=True)
            except Exception:
                pass
        # 复位点固定为「官方零位」= 14 个关节全 0（与遥控器复位一致）
        self.home = ZERO_POSE.copy()
        self.target = q.copy()
        self.current = q.copy()
        self.cmd = q.copy()
        try:
            self.rec = Recorder()
            print(f"[arm] 数据记录：{self.rec.path}", flush=True)
        except Exception as exc:
            print(f"[arm] 记录器不可用：{exc}", flush=True)
        self.vel = np.zeros(14)
        self.v_scale = 1.0
        self.speed = V_MAX
        self.grip_cmd = float(np.mean(s["grip_q2"]))
        self.grip_goal = self.grip_cmd
        self.grip_peak_tau = 0.0
        self.grasped = False
        self.grip_frozen = False
        self.grip_off = False        # True = 夹爪断电（电机禁用，可用手扳）
        self._probing = False
        self.grip_open = GRIP_OPEN
        self.grip_close = GRIP_CLOSE
        self.grip_full_cm = 4.5          # 全开时的指间距离（厘米），可在 gripper_range.json 里改
        self._load_grip_range()
        self._warn_at = 0.0
        self._last_sent = None
        self._tau_hist = []
        _mcfg = self.cfg["model"] if isinstance(self.cfg, dict) else self.cfg.model
        self.effort = _urdf_effort_limits(_mcfg["urdf"])
        if self.effort is not None:
            print("[arm] 关节力矩上限（URDF effort）："
                  + ", ".join(f"{n.split('_joint')[0]}={v:.0f}"
                              for n, v in zip(ARM_JOINT_NAMES, self.effort)), flush=True)
        # 反馈直接复用后端那一次订阅（再开一个订阅会让 DDS 接收线程开销翻倍）
        self.feedback = None
        self._last_fb = None
        print("[arm] 电机反馈：复用 lowstate 订阅（tau_est / 温度 / 电压）", flush=True)
        if not self.engaged:
            try:
                st = self.backend.read()
                if "tau35" in st:
                    self._last_fb = {k: st[k].copy() for k in ("tau35", "temp35", "vol35", "q35")}
            except Exception:
                pass
        threading.Thread(target=self._loop, daemon=True).start()

    def _load_grip_range(self):
        try:
            d = json.loads((ROOT / "gripper_range.json").read_text(encoding="utf-8"))
            lo, hi = float(d["min"]), float(d["max"])
            if hi > lo:
                self.grip_close, self.grip_open = lo, hi
                self.grip_full_cm = float(d.get("full_cm", 4.5))
                print(f"[arm] 夹爪行程（已测）: {lo:.4f} ~ {hi:.4f}"
                      f"，全开 {self.grip_full_cm:.1f} cm", flush=True)
        except Exception:
            pass

    def grip_cm(self, cm: float):
        """按厘米设定夹爪开口。"""
        span = self.grip_open - self.grip_close
        if span <= 1e-6:
            return None
        cm = float(np.clip(cm, 0.0, self.grip_full_cm))
        norm = self.grip_close + span * (cm / self.grip_full_cm)
        self.grip(norm)
        return {"cm": cm, "norm": round(norm, 4)}

    def grip_cm_now(self):
        span = self.grip_open - self.grip_close
        if span <= 1e-6:
            return None
        return round((self.grip_cmd - self.grip_close) / span * self.grip_full_cm, 2)

    def set_grip_full_cm(self, cm: float):
        cm = float(np.clip(cm, 1.0, 20.0))
        self.grip_full_cm = cm
        try:
            d = {}
            path = ROOT / "gripper_range.json"
            if path.exists():
                d = json.loads(path.read_text(encoding="utf-8"))
            d["full_cm"] = cm
            path.write_text(json.dumps(d, indent=1), encoding="utf-8")
        except Exception:
            pass
        return {"ok": True, "full_cm": cm}

    def _grip_tau(self):
        try:
            return float(np.max(np.abs(self._last_fb["tau35"][[31, 33]])))
        except Exception:
            return 0.0

    def _sweep_until_contact(self, sign, tau_stop, rate, max_travel):
        """朝一个方向慢慢走，直到力矩超阈值或走满行程。返回 (位置, 力矩, 行程)。"""
        step = sign * rate * 0.02
        travelled = 0.0
        tau = self._grip_tau()
        while travelled < max_travel:
            tau = self._grip_tau()
            if tau > tau_stop:
                break
            self.grip_cmd += step
            travelled += abs(step)
            time.sleep(0.02)
        return self.grip_cmd, tau, travelled

    def _release(self, sign, tau_release=0.10, max_back=0.12):
        """从接触状态退开，直到力矩回落（换向前必须先松力，否则一启动就超阈值）。"""
        step = sign * 0.002
        backed = 0.0
        while backed < max_back and self._grip_tau() > tau_release:
            self.grip_cmd += step
            backed += abs(step)
            time.sleep(0.02)
        time.sleep(0.15)
        return backed

    def probe_gripper(self, tau_stop=0.35, rate=0.08, max_travel=2.5):
        """自动测量夹爪行程：关到底 → 松力 → 开到底 → 松力。"""
        if not self.engaged or self.estopped:
            return {"ok": False, "why": "未接管或急停中"}
        if self.grip_off:
            return {"ok": False, "why": "夹爪处于断电状态，请先通电"}
        if self._last_fb is None:
            return {"ok": False, "why": "没有力矩反馈，无法安全测量"}
        self._probing = True
        out = {}
        try:
            pos_c, tau_c, trav_c = self._sweep_until_contact(-1, tau_stop, rate, max_travel)
            out["close"] = {"pos": round(pos_c, 4), "tau": round(tau_c, 3),
                            "travel": round(trav_c, 3)}
            self._release(+1)
            pos_o, tau_o, trav_o = self._sweep_until_contact(+1, tau_stop, rate, max_travel * 2)
            out["open"] = {"pos": round(pos_o, 4), "tau": round(tau_o, 3),
                           "travel": round(trav_o, 3)}
            self._release(-1)
            lo, hi = min(pos_c, pos_o), max(pos_c, pos_o)
            if hi - lo < 0.01:
                return {"ok": False, "why": f"行程异常（{lo:.3f}~{hi:.3f}），未保存", "detail": out}
            self.grip_close, self.grip_open = lo, hi
            with self.lock:
                self.grip_goal = self.grip_cmd
            try:
                (ROOT / "gripper_range.json").write_text(
                    json.dumps({"min": lo, "max": hi,
                                "measured_at": time.strftime("%Y-%m-%d %H:%M:%S")}, indent=1),
                    encoding="utf-8")
            except Exception:
                pass
            print(f"[arm] 夹爪行程实测: {lo:.4f} ~ {hi:.4f}", flush=True)
            return {"ok": True, "min": round(lo, 4), "max": round(hi, 4), "detail": out}
        finally:
            self._probing = False

    # --- connection -------------------------------------------------------
    def _connect(self):
        last = None
        for attempt in range(4):
            self.connect_state.update({"state": "connecting", "attempt": attempt + 1,
                                       "total": 4, "error": None})
            try:
                backend = G1DDSBackend(self.cfg)      # connect() 不可重用，必须新建
                backend.connect()
                self.backend = backend
                print(f"[arm] DDS 已连接（第 {attempt+1} 次）", flush=True)
                self.connect_state.update({"state": "connected", "attempt": attempt + 1,
                                           "error": None})
                return
            except Exception as exc:                  # noqa: BLE001
                last = exc
                print(f"[arm] DDS 连接第 {attempt+1} 次失败：{exc}", flush=True)
                self.connect_state.update({"state": "connecting", "attempt": attempt + 1,
                                           "error": str(exc)})
                time.sleep(4)
        self.connect_state.update({"state": "failed", "error": str(last)})
        raise last

    # --- engage / release -------------------------------------------------
    def engage_async(self):
        """点一下就走：立刻返回，后台线程去连/接管，页面轮询状态。"""
        if self.engaged:
            return {"status": "already"}
        if self._engage_thread is not None and self._engage_thread.is_alive():
            return {"status": "connecting"}
        self.connect_state = {"state": "connecting", "attempt": 0, "total": 4, "error": None}
        self._engage_thread = threading.Thread(target=self._engage_worker, daemon=True)
        self._engage_thread.start()
        return {"status": "connecting"}

    def _engage_worker(self):
        try:
            if self.backend is None:
                self._connect()
            self.engage()
            self.connect_state = {"state": "ready", "attempt": 0, "total": 4, "error": None}
            print("[arm] 异步接管完成", flush=True)
        except Exception as exc:                      # noqa: BLE001
            self.connect_state = {"state": "failed", "attempt": 0, "total": 4,
                                  "error": str(exc)}
            print(f"[arm] 异步接管失败：{exc}", flush=True)

    def engage(self):
        """先确认进入 debug 模式，再 acquire 并发布 —— 顺序不能反。

        反过来的话，在模式没切换成功时我们已经开始发指令，
        会和机器人自己的控制器抢电机（实测：肩 roll 力矩震荡 7 N·m、咔咔响）。
        """
        if self.engaged:
            return "already"
        if self.backend is None:
            self._connect()

        # ① 先切模式并确认（此期间一条指令都不发）
        mode, err = ensure_debug_mode()
        self.mode = mode
        print(f"[arm] 接管：mode={mode} {err or ''}", flush=True)
        if mode != "debug":
            print("[arm] 未能进入 debug 模式，放弃接管", flush=True)
            hand_back_to_robot()
            raise RuntimeError(f"无法进入 debug 模式（当前 {mode}），未接管")

        # ② 读取当前姿态作为种子
        s = self.backend.read()
        q = np.array(s["q14"], float)
        with self.lock:
            self.current = q.copy()
            self.target = q.copy()
            self.cmd = q.copy()
            self.vel = np.zeros(14)
            self._last_sent = None

        # ③ acquire 后立刻开始发布（下一条 tick 内，滞后 < 10 ms）
        self.backend.acquire(s["grip_q2"], s["waist_q3"], self.kin.gravity(q))
        self.engaged = True
        time.sleep(0.05)
        try:
            s = self.backend.read()
            q = np.array(s["q14"], float)
            with self.lock:
                self.current = q.copy()
                self.target = q.copy()
                self.cmd = q.copy()
        except Exception:
            pass
        return mode

    def release(self):
        self.engaged = False
        try:
            self.backend.hold()
        except Exception:
            pass
        self.backend = None          # 释放后后端已锁死，下次接管必须重新连接
        hand_back_to_robot()

    # --- control ----------------------------------------------------------
    def _loop(self):
        stall_since = None
        stale_since = None
        while self.alive:
            if not self.engaged:
                time.sleep(0.05)
                continue
            try:
                state = self.backend.read()
                measured = np.array(state["q14"], float)
                stale_since = None
                if "tau35" in state:
                    self._last_fb = {k: state[k].copy()
                                     for k in ("tau35", "temp35", "vol35", "q35")}
            except Exception:
                stale_since = stale_since or time.monotonic()
                if time.monotonic() - stale_since > 0.5:
                    self._fault("DDS 数据中断超过 0.5s（网线/机器人掉线）")
                    return
                time.sleep(TICK)
                continue
            with self.lock:
                target = self.target.copy()
                cmd = self.cmd.copy()
                stopped = self.estopped

            if stopped:
                # 急停：冻结在实测位置，继续以重力补偿保持，绝不下坠
                cmd = measured.copy()
                self._publish_hold(cmd, state)
                with self.lock:
                    self.cmd = cmd
                    self.current = measured
                time.sleep(TICK)
                continue

            # 梯形速度曲线：加速 → 匀速 → 减速，速度连续、无阶跃
            d = target - cmd
            vmax = V_MAX * self.v_scale
            v_des = np.sign(d) * np.minimum(vmax, np.sqrt(2 * A_MAX * np.abs(d)))
            v = np.clip(v_des, self.vel - A_MAX * TICK, self.vel + A_MAX * TICK)
            step = np.clip(v * TICK, -np.abs(d), np.abs(d))
            cmd = cmd + step
            self.vel = np.where(np.abs(d) > 1e-6, step / TICK, 0.0)
            cmd = np.clip(cmd, measured - LEAD, measured + LEAD)   # 防饱和

            err = float(np.max(np.abs(cmd - measured)))
            now = time.monotonic()
            if err > LEAD * STALL_FRAC:
                stall_since = stall_since or now
                if now - stall_since > STALL_SEC:
                    self._fault(f"检测到堵转：指令与实测相差 {err:.3f} rad 超过 {STALL_SEC}s")
                    return
            else:
                stall_since = None

            if not self._publish_hold(cmd, state):
                return
            with self.lock:
                self.cmd = cmd
                self.current = measured               # 界面显示实测，不是指令

            # ---- 异常检测（力矩 / 温度 / 电压 / 夹爪跟随） ----
            if self._last_fb is not None:
                snap = self._last_fb
                if True:
                    gq = float(np.mean(snap["q35"][list(GRIP_SLOTS)]))
                    tau_arm = np.asarray(snap["tau35"][15:29], float)
                    # 振荡检测：位置几乎不动但力矩剧烈起伏 → 多半是两套控制器在抢
                    self._tau_hist.append(tau_arm.copy())
                    if len(self._tau_hist) > 20:
                        self._tau_hist.pop(0)
                    if len(self._tau_hist) >= 10:
                        h = np.asarray(self._tau_hist)
                        ptp = float(np.max(np.ptp(h, axis=0)))
                        if ptp > OSC_TAU:
                            bad_j = ARM_JOINT_NAMES[int(np.argmax(np.ptp(h, axis=0)))]
                            self._fault(f"异常停机：{bad_j} 力矩在 0.2 秒内起伏 "
                                        f"{ptp:.1f} N·m，疑似控制器冲突/关节振荡")
                            return
                    limit = 25.0   # 读不到 URDF 时的保守回退
                    if self.effort is not None:
                        ratio = float(np.max(np.abs(tau_arm) / self.effort))
                        limit = EFFORT_SAFETY
                        if ratio > limit:
                            idx = int(np.argmax(np.abs(tau_arm) / self.effort))
                            self._fault(f"异常停机：{ARM_JOINT_NAMES[idx]} 力矩 "
                                        f"{tau_arm[idx]:.1f} 达机械上限 "
                                        f"{self.effort[idx]:.0f} 的 {ratio*100:.0f}%")
                            return
                    # 夹爪断电时不再检查"未跟随"（本来就该松着）
                    bad, why, level = Anomaly.check(
                        tau_arm, np.asarray(snap["temp35"][15:29], float),
                        np.asarray(snap["vol35"][15:29], float),
                        self.grip_cmd if not self.grip_off else gq,
                        gq, 1e9, TEMP_MAX, VOLT_MIN)
                    if bad and level == "fault":
                        self._fault(f"异常停机：{why}")
                        return
                    if bad and time.monotonic() - self._warn_at > 2.0:
                        self._warn_at = time.monotonic()
                        print(f"[arm] 警告：{why}", flush=True)
                    if self.rec is not None:
                        self.rec.log(q=measured, cmd=cmd, grip_cmd=self.grip_cmd,
                                     grip_q=gq, grip_tau=np.asarray(snap["tau35"][[31, 33]], float),
                                     tau=tau_arm,
                                     temp=np.asarray(snap["temp35"][15:29], float))
            time.sleep(TICK)

    def _publish_hold(self, cmd, state):
        try:
            prev_grip = self.grip_cmd
            grip_cmd, _tau = self._step_grip()
            grip_changed = abs(grip_cmd - prev_grip) > 1e-6
            try:
                self.backend._grip_target = np.array(state["grip_q2"], float)
            except Exception:
                pass
            if not self.grip_off:
                for slot in GRIP_SLOTS:
                    self.backend._message.motor_cmd[slot].q = grip_cmd
            # 指令没有实质变化就不重发：后端发布线程本来就在 250 Hz 持续推送，
            # 每 10 ms 重设一次力矩会造成力矩阶跃（表现为小幅抽搐）。
            if (self._last_sent is None
                    or float(np.max(np.abs(cmd - self._last_sent))) > 2e-4
                    or grip_changed):
                self.backend.command(cmd, self.kin.gravity(cmd))
                self._last_sent = cmd.copy()
            return True
        except Exception as exc:                      # noqa: BLE001
            self._fault(str(exc))
            return False

    # --- emergency stop ---------------------------------------------------
    def estop(self, reason="手动急停"):
        with self.lock:
            if self.estopped:
                return
            self.estopped = True
            self.estop_reason = reason
            measured = self.current.copy()
            self.target = measured.copy()
            self._last_sent = None
        print(f"[arm] E-STOP: {reason}", flush=True)

    def resume(self):
        with self.lock:
            if not self.estopped:
                return
            self.estopped = False
            self.estop_reason = None
            self.target = self.current.copy()
            self.cmd = self.current.copy()
            self.last_heartbeat = time.monotonic()
        print("[arm] 急停已解除", flush=True)

    def heartbeat(self):
        self.last_heartbeat = time.monotonic()

    def set_heartbeat(self, enabled: bool):
        self.hb_enabled = bool(enabled)
        self.last_heartbeat = time.monotonic()
        print(f"[arm] 心跳看门狗：{'开启' if enabled else '关闭'}", flush=True)
        return {"ok": True, "on": bool(enabled)}

    def watchdog_loop(self):
        while self.alive:
            time.sleep(0.5)
            if (self.hb_enabled and self.engaged and not self.estopped
                    and time.monotonic() - self.last_heartbeat > HB_TIMEOUT):
                self.estop("界面心跳丢失（看门狗已开启）")

    def _fault(self, text):
        self.fault = text
        self.engaged = False
        print(f"[arm] {text}", flush=True)
        try:
            self.backend.hold()
        except Exception:
            pass
        self.backend = None
        hand_back_to_robot()

    # --- commands ---------------------------------------------------------
    def q(self):
        with self.lock:
            return self.current.copy()

    def _clamp_limits(self, want):
        """只按关节硬限位裁剪。防饱和由控制环对「指令」做，不能夹目标，
        否则长距离移动（尤其复位）会只走 LEAD 那么远就停住。"""
        lo = np.asarray(self.kin.lower, float) + LIMIT_MARGIN
        hi = np.asarray(self.kin.upper, float) - LIMIT_MARGIN
        return np.clip(np.asarray(want, float), lo, hi)

    def jog(self, deltas):
        if not self.engaged:
            return {"_": "未接管"}
        if self.estopped:
            return {"_": "急停中"}
        with self.lock:
            self.v_scale = 1.0
            measured = self.current.copy()
            candidate = self.target.copy()
            applied = {}
            for name, raw in (deltas or {}).items():
                if name not in ARM_JOINT_NAMES:
                    continue
                i = ARM_JOINT_NAMES.index(name)
                d = float(np.clip(float(raw), -MAX_STEP, MAX_STEP))
                lo_hard = float(self.kin.lower[i]) + LIMIT_MARGIN
                hi_hard = float(self.kin.upper[i]) - LIMIT_MARGIN
                want = float(np.clip(candidate[i] + d, lo_hard, hi_hard))
                lo = max(lo_hard, measured[i] - LEAD)
                hi = min(hi_hard, measured[i] + LEAD)
                applied[i] = float(np.clip(want, lo, hi))
                candidate[i] = applied[i]
        # 碰撞检查放在锁外：单次约 35ms，不能挡住控制环
        bad = None
        if self.collision is None:
            bad = "自碰撞检测不可用"
        else:
            try:
                ok, _d, why = self.collision.check(candidate)
                if not ok:
                    bad = why
            except Exception:
                bad = "碰撞检测异常"
        out = {}
        if bad:
            for i in applied:
                out[ARM_JOINT_NAMES[i]] = 0.0
            out["_rejected"] = bad
            return out
        with self.lock:
            for i, v in applied.items():
                out[ARM_JOINT_NAMES[i]] = round(v - self.target[i], 4)
                self.target[i] = v
        return out

    def goto(self, q14):
        if not self.engaged or self.estopped:
            return False, "未接管或急停中"
        with self.lock:
            start = self.current.copy()
            self.v_scale = 0.6
            target = self._clamp_limits(q14)
        ok, why = self._collision_ok(start, target)
        if not ok:
            return False, why
        with self.lock:
            self.target = target
            self.vel = np.zeros(14)
        return True, "ok"

    def _collision_ok(self, start, target):
        if self.collision is None:
            return False, "自碰撞检测不可用，拒绝运动"
        margin = float(self.collision.margin)
        try:
            # 起点自身的安全间距：如果它已经低于安全线，就不能要求整条路径
            # 都达标（那样机器人会被卡死，连退出来的动作都会被拒）。
            ok0, d0, _w0 = self.collision.check(start)
            floor = margin if d0 >= margin else max(0.0, d0 - 0.002)
            span = float(np.max(np.abs(np.asarray(target, float) - np.asarray(start, float))))
            steps = int(np.clip(span / 0.04, 4, 80))
            worst = float("inf")
            worst_t = 0.0
            for t in np.linspace(0.0, 1.0, max(2, steps)):
                q = np.asarray(start, float) + (np.asarray(target, float)
                                                - np.asarray(start, float)) * t
                _ok, d, why = self.collision.check(q)
                if d < worst:
                    worst, worst_t = d, t
                if d < floor:
                    tag = "已低于安全线" if d0 < margin else "碰撞风险"
                    return False, (f"{tag}：路径 t={t:.2f} 处间距 {d*1000:.0f} mm"
                                   f"（起点 {d0*1000:.0f} mm，要求不低于 {floor*1000:.0f} mm）")
        except Exception as exc:
            return False, f"碰撞检测异常：{type(exc).__name__}"
        if d0 < margin:
            return True, (f"允许（起点仅 {d0*1000:.0f} mm，低于安全线；"
                          f"路径最低 {worst*1000:.0f} mm，未变差）")
        return True, f"ok（最低 {worst*1000:.0f} mm）"

    def pose_target(self, name):
        """返回具名姿态的关节目标。"""
        if name == "zero":
            return ZERO_POSE.copy()
        if name == "dangle":
            return np.asarray(self.dangle, float).copy()
        return None

    def precheck(self, name):
        """只检查不运动：返回 (是否可以, 说明)。"""
        target = self.pose_target(name)
        if target is None:
            return False, f"未知姿态 {name}"
        with self.lock:
            start = self.current.copy()     # 未接管时用最近一次实测姿态做检查
        target = self._clamp_limits(target)
        return self._collision_ok(start, target)

    def go_pose(self, name):
        """缓慢移动到具名姿态（先整条路径碰撞检查）。"""
        if not self.engaged or self.estopped:
            return False, "未接管或急停中"
        target = self.pose_target(name)
        if target is None:
            return False, f"未知姿态 {name}"
        with self.lock:
            start = self.current.copy()
            self.v_scale = 0.6
            target = self._clamp_limits(target)
        ok, why = self._collision_ok(start, target)
        if not ok:
            return False, why
        with self.lock:
            self.target = target
            self.vel = np.zeros(14)
        return True, "ok"

    def go_home(self):
        return self.go_pose("zero")

    def go_zero(self):
        return self.go_pose("zero")

    def go_dangle(self):
        return self.go_pose("dangle")

    def grip(self, value):
        val = float(np.clip(value, self.grip_close - 0.001, self.grip_open + 0.001))
        self.grip_val = val
        if not self.engaged or self.estopped:
            return val
        with self.lock:
            self.grip_goal = val
            self.grip_peak_tau = 0.0
            self.grasped = False
            self.grip_frozen = False      # 新命令必须解冻，否则永远动不了
        return val

    def _step_grip(self):
        """每拍推进夹爪，限速 + 力矩保护。返回 (cmd, tau)。"""
        tau = self._grip_tau()
        with self.lock:
            if self.estopped or self.grip_off or self._probing:
                return self.grip_cmd, tau
            goal, cmd = self.grip_goal, self.grip_cmd
            self.grip_peak_tau = max(self.grip_peak_tau, tau)
            # 力控：无论张开还是闭合，力矩超限就冻结，绝不再加力
            if tau > GRIP_TAU_MAX and not self.grip_frozen:
                self.grip_goal = cmd
                self.grip_frozen = True
                goal = cmd
            elif goal < cmd and tau > GRIP_TAU_GRASP:
                self.grasped = True
            step = float(np.clip(goal - cmd, -GRIP_RATE * TICK, GRIP_RATE * TICK))
            self.grip_cmd = cmd + step
            out = self.grip_cmd
        return out, tau

    def set_grip_power(self, on: bool):
        """夹爪通电/断电。断电 = 电机 mode=0，两指可自由扳动。"""
        if not self.engaged or getattr(self, "estopped", False):
            return {"ok": False, "on": not self.grip_off, "why": "未接管或急停中"}
        if self.backend is None:
            return {"ok": False, "on": not self.grip_off, "why": "后端未连接"}
        self.grip_off = not on
        try:
            gains = self.cfg["gains"]
            kp = float(gains["gripper_kp"][0])
            kd = float(gains["gripper_kd"][0])
        except Exception:
            kp, kd = 5.0, 0.05
        with self.lock:
            for slot in GRIP_SLOTS:
                try:
                    m = self.backend._message.motor_cmd[slot]
                except Exception:
                    continue
                if on:
                    m.mode = 1
                    m.kp, m.kd = kp, kd
                    m.q = float(self.grip_cmd)
                    m.dq, m.tau = 0.0, 0.0
                else:
                    m.mode = 0
                    m.kp, m.kd, m.tau = 0.0, 0.0, 0.0
            if on:
                self.grip_cmd = float(np.mean(self.backend.read()["grip_q2"]))
                self.grip_goal = self.grip_cmd
        print(f"[arm] 夹爪{'通电' if on else '断电'}", flush=True)
        return {"ok": True, "on": bool(on)}

    def grip_state(self):
        with self.lock:
            return {"cmd": round(self.grip_cmd, 4), "goal": round(self.grip_goal, 4),
                    "peak_tau": round(self.grip_peak_tau, 3),
                    "grasped": self.grasped, "frozen": self.grip_frozen,
                    "power": not self.grip_off,
                    "cm": self.grip_cm_now(), "full_cm": self.grip_full_cm,
                    "min": round(self.grip_close, 4), "max": round(self.grip_open, 4)}

    def busy(self):
        with self.lock:
            return float(np.max(np.abs(self.target - self.current)))

    def shutdown(self):
        self.alive = False
        self.engaged = False
        try:
            self.backend.hold()
        except Exception:
            pass
        if self.rec is not None:
            try:
                self.rec.close()
            except Exception:
                pass
        hand_back_to_robot()


class SimArm:
    """无硬件预览：同样的接口，不发任何 DDS。"""

    class _Kin:
        lower = np.full(14, -2.8)
        upper = np.full(14, 2.8)

    def __init__(self):
        self.lock = threading.Lock()
        self.kin = self._Kin()
        self.home = np.zeros(14)
        self.current = np.zeros(14)
        self.target = np.zeros(14)
        self.cmd = np.zeros(14)
        self.vel = np.zeros(14)
        self.v_scale = 1.0
        self.fault = None
        self.mode = "sim"
        self.engaged = True
        self.estopped = False
        self.estop_reason = None
        self.last_heartbeat = time.monotonic()
        self.grip_val = GRIP_OPEN
        self.grip_off = False
        self.dangle = np.zeros(14)
        self.alive = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.alive:
            with self.lock:
                d = self.target - self.current
                vmax = V_MAX * self.v_scale
                v_des = np.sign(d) * np.minimum(vmax, np.sqrt(2 * A_MAX * np.abs(d)))
                v = np.clip(v_des, self.vel - A_MAX * TICK, self.vel + A_MAX * TICK)
                step = np.clip(v * TICK, -np.abs(d), np.abs(d))
                self.current = self.current + step
                self.vel = np.where(np.abs(d) > 1e-9, step / TICK, 0.0)
            time.sleep(TICK)

    def engage_async(self):
        """点一下就走：立刻返回，后台线程去连/接管，页面轮询状态。"""
        if self.engaged:
            return {"status": "already"}
        if self._engage_thread is not None and self._engage_thread.is_alive():
            return {"status": "connecting"}
        self.connect_state = {"state": "connecting", "attempt": 0, "total": 4, "error": None}
        self._engage_thread = threading.Thread(target=self._engage_worker, daemon=True)
        self._engage_thread.start()
        return {"status": "connecting"}

    def _engage_worker(self):
        try:
            if self.backend is None:
                self._connect()
            self.engage()
            self.connect_state = {"state": "ready", "attempt": 0, "total": 4, "error": None}
            print("[arm] 异步接管完成", flush=True)
        except Exception as exc:                      # noqa: BLE001
            self.connect_state = {"state": "failed", "attempt": 0, "total": 4,
                                  "error": str(exc)}
            print(f"[arm] 异步接管失败：{exc}", flush=True)

    def engage(self):
        self.engaged = True
        return "sim"

    def release(self):
        self.engaged = False

    def estop(self, reason="手动急停"):
        self.estopped = True
        self.estop_reason = reason
        with self.lock:
            self.target = self.current.copy()

    def resume(self):
        self.estopped = False
        self.estop_reason = None
        with self.lock:
            self.target = self.current.copy()
            self.cmd = self.current.copy()

    def heartbeat(self):
        self.last_heartbeat = time.monotonic()

    def set_heartbeat(self, enabled: bool):
        self.hb_enabled = bool(enabled)
        self.last_heartbeat = time.monotonic()
        return {"ok": True, "on": bool(enabled)}

    def watchdog_loop(self):
        while self.alive:
            time.sleep(0.5)
            if (getattr(self, "hb_enabled", False) and self.engaged and not self.estopped
                    and time.monotonic() - self.last_heartbeat > HB_TIMEOUT):
                self.estop("界面心跳丢失")

    def q(self):
        with self.lock:
            return self.current.copy()

    def jog(self, deltas):
        if not self.engaged:
            return {"_": "未接管"}
        if self.estopped:
            return {"_": "急停中"}
        if self.estopped:
            return {"_": "急停中"}
        out = {}
        with self.lock:
            self.v_scale = 1.0
            for name, raw in (deltas or {}).items():
                if name not in ARM_JOINT_NAMES:
                    continue
                i = ARM_JOINT_NAMES.index(name)
                d = float(np.clip(float(raw), -MAX_STEP, MAX_STEP))
                new = float(np.clip(self.target[i] + d, -2.8, 2.8))
                out[name] = round(new - self.target[i], 4)
                self.target[i] = new
        return out

    def goto(self, q14):
        if not self.engaged or self.estopped:
            return False, "未接管或急停中"
        with self.lock:
            self.v_scale = 0.6
            self.target = np.clip(np.asarray(q14, float), -2.8, 2.8)
        return True, "ok"

    def pose_target(self, name):
        if name == "zero":
            return np.zeros(14)
        if name == "dangle":
            return np.asarray(getattr(self, "dangle", np.zeros(14)), float).copy()
        return None

    def precheck(self, name):
        if not self.engaged:
            return False, "未接管"
        return True, "ok（模拟）"

    def go_pose(self, name):
        if not self.engaged or self.estopped:
            return False, "未接管或急停中"
        target = self.pose_target(name)
        if target is None:
            return False, f"未知姿态 {name}"
        with self.lock:
            self.target = target
        return True, "ok"

    def go_home(self):
        return self.go_pose("zero")

    def go_zero(self):
        return self.go_pose("zero")

    def go_dangle(self):
        return self.go_pose("dangle")

    def grip(self, value):
        self.grip_val = float(value)
        return self.grip_val

    def set_grip_power(self, on: bool):
        """夹爪通电/断电。断电 = 电机 mode=0，两指可自由扳动。"""
        if not self.engaged or getattr(self, "estopped", False):
            return {"ok": False, "on": not self.grip_off, "why": "未接管或急停中"}
        if self.backend is None:
            return {"ok": False, "on": not self.grip_off, "why": "后端未连接"}
        self.grip_off = not on
        try:
            gains = self.cfg["gains"]
            kp = float(gains["gripper_kp"][0])
            kd = float(gains["gripper_kd"][0])
        except Exception:
            kp, kd = 5.0, 0.05
        with self.lock:
            for slot in GRIP_SLOTS:
                try:
                    m = self.backend._message.motor_cmd[slot]
                except Exception:
                    continue
                if on:
                    m.mode = 1
                    m.kp, m.kd = kp, kd
                    m.q = float(self.grip_cmd)
                    m.dq, m.tau = 0.0, 0.0
                else:
                    m.mode = 0
                    m.kp, m.kd, m.tau = 0.0, 0.0, 0.0
            if on:
                self.grip_cmd = float(np.mean(self.backend.read()["grip_q2"]))
                self.grip_goal = self.grip_cmd
        print(f"[arm] 夹爪{'通电' if on else '断电'}", flush=True)
        return {"ok": True, "on": bool(on)}

    def set_grip_power(self, on: bool):
        self.grip_off = not on
        return {"ok": True, "on": bool(on)}

    def grip_cm(self, cm):
        self.grip_val = float(cm)
        return {"cm": float(cm), "norm": float(cm)}

    def grip_cm_now(self):
        return round(float(getattr(self, "grip_val", 0.0)), 2)

    def set_grip_full_cm(self, cm):
        return {"ok": True, "full_cm": float(cm)}

    def grip_state(self):
        return {"cmd": round(self.grip_val, 4), "goal": round(self.grip_val, 4),
                "peak_tau": 0.0, "grasped": False, "frozen": False,
                "power": not getattr(self, "grip_off", False),
                "cm": self.grip_cm_now(), "full_cm": 4.5, "min": 0.0, "max": 4.5}

    def busy(self):
        with self.lock:
            return float(np.max(np.abs(self.target - self.current)))

    def shutdown(self):
        self.alive = False


# ---------------------------------------------------------------------- cameras
class Camera:
    def __init__(self, url):
        self.url = url
        self.latest = None
        self.count = 0
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        import zmq
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.RCVTIMEO, 2000)
        sock.connect(self.url)
        while True:
            try:
                data = sock.recv()
            except Exception:
                time.sleep(0.2)
                continue
            with self.lock:
                self.latest = data
                self.count += 1

    def jpeg(self):
        with self.lock:
            return self.latest


CAMS = {name: Camera(url) for name, (url, _) in CAM_SOURCES.items()}


def eye_jpeg(half):
    import cv2
    jpg = CAMS["head"].jpeg()
    if jpg is None:
        return None
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    h, w = img.shape[:2]
    if w > h:
        img = img[:, : w // 2] if half == "left" else img[:, w // 2:]
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return buf.tobytes() if ok else None


def snapshot_b64(which="head", width=900):
    import cv2
    raw = eye_jpeg("left") if which == "head" else (CAMS[which].jpeg() if which in CAMS else None)
    if raw is None:
        return None
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    h, w = img.shape[:2]
    img = cv2.resize(img, (width, int(width * h / w)))
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return base64.b64encode(buf.tobytes()).decode() if ok else None


# ------------------------------------------------------------------------- store
class Store:
    def __init__(self):
        self.lock = threading.Lock()
        self.waypoints = {}
        self.sequences = {}
        self.agent = {"running": False, "log": [], "task": "", "done": False}
        self.arm = None
        self.playing = False
        self.load()

    def load(self):
        if not LIB.exists():
            return
        try:
            data = json.loads(LIB.read_text(encoding="utf-8"))
            self.waypoints = data.get("waypoints", {})
            self.sequences = data.get("sequences", {})
        except Exception:
            pass

    def save(self):
        with self.lock:
            payload = {"waypoints": self.waypoints, "sequences": self.sequences}
        LIB.write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8")

    def log(self, kind, text):
        with self.lock:
            self.agent["log"].append({"t": time.strftime("%H:%M:%S"), "kind": kind,
                                      "text": str(text)[:600]})
            self.agent["log"] = self.agent["log"][-400:]


S = Store()
MOTION = {"allow": True}


# ------------------------------------------------------------------------- agent
TOOLS = [
    {"type": "function", "function": {
        "name": "goto_waypoint",
        "description": "移动到示教航点，并恢复该航点记录的夹爪开合。",
        "parameters": {"type": "object", "required": ["name"], "properties": {
            "name": {"type": "string"},
            "with_grip": {"type": "boolean",
                          "description": "是否同时恢复航点记录的夹爪状态，默认 true"}}}}},
    {"type": "function", "function": {
        "name": "goto_waypoints",
        "description": "按顺序依次移动到多个示教航点，并恢复每个航点记录的夹爪开合。",
        "parameters": {"type": "object", "required": ["names"], "properties": {
            "names": {"type": "array", "items": {"type": "string"},
                      "description": "航点名列表，按执行顺序排列"},
            "with_grip": {"type": "boolean",
                          "description": "是否恢复各航点的夹爪状态，默认 true"}}}}},
    {"type": "function", "function": {
        "name": "jog_multi",
        "description": "同时微调多个关节（相对当前目标），每步建议 0.05~0.15 rad。",
        "parameters": {"type": "object", "required": ["deltas"], "properties": {
            "deltas": {"type": "object", "additionalProperties": {"type": "number"}}}}}},
    {"type": "function", "function": {
        "name": "gripper", "description": "开合夹爪。",
        "parameters": {"type": "object", "required": ["action"], "properties": {
            "action": {"type": "string", "enum": ["open", "close"]}}}}},
    {"type": "function", "function": {
        "name": "look_gripper",
        "description": "再看一眼左腕相机（夹爪特写），确认是否夹住、是否对准孔。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "wait", "description": "等待动作完成。",
        "parameters": {"type": "object", "required": ["seconds"], "properties": {
            "seconds": {"type": "number"}}}}},
    {"type": "function", "function": {
        "name": "reset", "description": "回到启动位（半速，不避障）。",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "finish", "description": "任务结束并说明结果。",
        "parameters": {"type": "object", "required": ["summary"], "properties": {
            "summary": {"type": "string"}}}}},
]


def prune_images(messages):
    """历史消息里的图片换成占位符：每轮都带图会让请求体积爆炸
    （实测第二轮就被服务端断开：RemoteDisconnected）。"""
    for msg in messages[:-1]:
        c = msg.get("content")
        if isinstance(c, list):
            msg["content"] = [
                {"type": "text", "text": "[此轮图片已省略]"} if part.get("type") == "image_url"
                else part for part in c]


def ask(messages):
    os.environ.setdefault("HTTPS_PROXY", "http://127.0.0.1:7890")
    os.environ.setdefault("HTTP_PROXY", "http://127.0.0.1:7890")
    body = json.dumps({"model": MODEL, "messages": messages, "tools": TOOLS,
                       "tool_choice": "auto", "max_tokens": 800}).encode()
    req = urllib.request.Request(
        f"{API_BASE}/chat/completions", data=body,
        headers={"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"})
    last = None
    for attempt in range(2):
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                return json.loads(r.read())["choices"][0]["message"]
        except Exception as exc:                     # noqa: BLE001
            last = exc
            if attempt == 0:
                time.sleep(2)
                continue
            raise last


SYSTEM = (
    "你控制一台 Unitree G1-D 的左臂。每轮收到两张图：头部左目（全局）+ 左腕（夹爪特写）。\n"
    "动作优先级：goto_waypoint（示教航点，最快）> jog_multi（多关节微调）> gripper。\n"
    "多步任务：如果任务要求经过多个航点（例如「1 到 4」），必须调用 goto_waypoints，\n"
    "把全部航点名一次传进去（如 \"names\": [\"1\",\"2\",\"3\",\"4\"]），系统会按顺序执行。\n"
    "绝对不要只执行第一步就结束。\n"
    "航点会自带夹爪开合状态（录制时记下的），执行航点时系统会自动恢复，不需要额外调 gripper。\n"
    "规则：\n"
    "- 夹爪闭合后先 look_gripper 确认物体是否在两指之间，再抬升。\n"
    "- 每次动作后 wait 1~3 秒再判断。\n"
    "- 只用给定的关节名和航点名。\n"
    "- 完成或做不到时调用 finish，说明卡在哪一步。\n"
    "关节名：{joints}\n可用航点：{waypoints}\n可用轨迹：{sequences}\n"
)


def settle(arm, timeout=8.0):
    end = time.time() + timeout
    while time.time() < end and arm.busy() > 0.01:
        time.sleep(0.05)
    time.sleep(0.4)


def settle_grip(arm, timeout=40.0):
    """等夹爪走到目标（慢速时可能要 30 秒），被力控冻结也算结束。"""
    end = time.time() + timeout
    while time.time() < end:
        try:
            st = arm.grip_state()
        except Exception:
            return
        if st.get("frozen"):
            return
        if abs(float(st.get("cmd", 0.0)) - float(st.get("goal", 0.0))) < 1e-4:
            return
        time.sleep(0.2)


SUCCESS_EPS = 0.030       # 到位阈值（rad，约 1.7°）；稳态跟踪误差本来就有几毫弧度


def describe_call(name, a):
    if name == "goto_waypoint":
        return f"移动到航点「{a.get('name','?')}」"
    if name == "goto_waypoints":
        return f"依次执行 {a.get('names') or []}"
    if name == "jog_multi":
        return f"微调 {a.get('deltas') or {}}"
    if name == "gripper":
        return f"夹爪{'张开' if a.get('action')=='open' else '闭合'}"
    if name == "reset":
        return "回零位"
    if name == "wait":
        return f"等待 {a.get('seconds')} 秒"
    if name == "look_gripper":
        return "查看夹爪特写"
    if name == "finish":
        return "任务结束"
    return name


def _motion_ok(arm):
    """到位判定：目标与实测的最大差。"""
    try:
        return float(arm.busy()) <= SUCCESS_EPS
    except Exception:
        return False


def run_agent(task, turns, allow_motion):
    arm = S.arm
    t_start = time.time()
    moved_any = False
    S.agent.update({"running": True, "done": False, "task": task})
    try:
        if not getattr(arm, "engaged", False):
            S.log("error", "手臂未接管，拒绝执行任务")
            return
        with S.lock:
            wps = {k: {"grip": v["grip"]} for k, v in S.waypoints.items()}
            seqs = {k: v["waypoints"] for k, v in S.sequences.items()}
        system = SYSTEM.format(joints=", ".join(ARM_JOINT_NAMES),
                               waypoints=json.dumps(wps, ensure_ascii=False),
                               sequences=json.dumps(seqs, ensure_ascii=False))
        messages = [{"role": "system", "content": system},
                    {"role": "user", "content": f"任务：{task}"}]
        for turn in range(turns):
            if not S.agent["running"]:
                S.log("stop", "用户中止")
                return
            prune_images(messages)
            head = snapshot_b64("head", 900)
            wrist = snapshot_b64("lwrist", 640)
            content = [{"type": "text", "text": f"第 {turn+1} 轮，当前关节角 "
                                                f"{json.dumps(np.round(arm.q(), 3).tolist())}"}]
            if head:
                content += [{"type": "text", "text": "头部左目："},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{head}"}}]
            if wrist:
                content += [{"type": "text", "text": "左腕/夹爪："},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{wrist}"}}]
            messages.append({"role": "user", "content": content})
            if head and wrist:
                S.log("info", "已发送相机图像：头部 900px + 左腕 640px")
            elif head:
                S.log("info", "已发送相机图像：头部 900px（腕部无画面）")
            else:
                S.log("warn", "没有相机画面，仅发送关节角")
            S.log("api", "调用 API（gpt-6-astra）…")
            t_api = time.time()
            msg = ask(messages)
            S.log("api", f"API 响应正常（{time.time()-t_api:.1f}s）")
            messages.append(msg)
            if msg.get("content"):
                S.log("think", msg["content"][:400])
            calls = msg.get("tool_calls") or []
            if not calls:
                S.log("warn", "模型没有调用工具，结束")
                return
            plans = [describe_call(c["function"]["name"],
                                   json.loads(c["function"].get("arguments") or "{}")
                                   if c["function"].get("arguments") else {})
                     for c in calls]
            S.log("info", f"计划执行 {len(calls)} 个动作：" + " → ".join(plans))
            for step_i, call in enumerate(calls, 1):
                name = call["function"]["name"]
                try:
                    a = json.loads(call["function"].get("arguments") or "{}")
                except json.JSONDecodeError:
                    a = {}
                if name == "finish":
                    S.log("finish", a.get("summary", ""))
                    return
                if not allow_motion and name not in ("look_gripper", "wait"):
                    S.log("dry", f"{name} {a}")
                    result = "只读模式，未执行"
                elif name == "goto_waypoint":
                    with S.lock:
                        wp = S.waypoints.get(a.get("name", ""))
                    if not wp:
                        result = f"没有航点 {a.get('name')}"
                        S.log("warn", result)
                    else:
                        ok, why = arm.goto(wp["q"])
                        if not ok:
                            S.log("warn", f"航点 {a['name']} 被拒绝：{why}")
                            result = f"拒绝：{why}"
                        else:
                            S.log("move", f"→ 航点 {a['name']}")
                            settle(arm)
                            if a.get("with_grip", True) and "grip" in wp:
                                arm.grip(float(wp["grip"]))
                                settle_grip(arm, timeout=10)
                                S.log("grip", f"夹爪恢复到 {float(wp['grip']):.3f}")
                            result = f"已到位 {a['name']}"
                elif name == "goto_waypoints":
                    names = [str(x) for x in (a.get("names") or [])]
                    if not names:
                        result = "没有提供航点"
                    else:
                        ok_all, done = True, []
                        for i, nm in enumerate(names, 1):
                            with S.lock:
                                wp = S.waypoints.get(nm)
                            if not wp:
                                ok_all = False
                                S.log("step", f"  {i}/{len(names)} ❌ 没有航点「{nm}」")
                                continue
                            S.log("step", f"  {i}/{len(names)} 执行中：→ 「{nm}」")
                            okp, why = arm.goto(wp["q"])
                            if not okp:
                                ok_all = False
                                S.log("step", f"  {i}/{len(names)} ❌ 被拒绝：{why}")
                                continue
                            settle(arm)
                            if a.get("with_grip", True) and "grip" in wp:
                                arm.grip(float(wp["grip"]))
                                settle_grip(arm, timeout=10)
                            okp = _motion_ok(arm)
                            S.log("step", f"  {i}/{len(names)} {'✅ 成功' if okp else '⚠️ 有误差'}："
                                          f"「{nm}」 夹爪 {float(wp.get('grip', 0)):.2f} "
                                          f"err={arm.busy():.3f} rad")
                            if okp:
                                done.append(nm)
                        result = f"完成 {len(done)}/{len(names)}：{done}"
                        if not ok_all:
                            result += "（有失败项）"
                elif name == "jog_multi":
                    moved = arm.jog(a.get("deltas") or {})
                    S.log("move", f"微调 {moved}")
                    settle(arm)
                    result = f"已下发 {moved}"
                elif name == "gripper":
                    val = GRIP_OPEN if a.get("action") == "open" else GRIP_CLOSE
                    arm.grip(val)
                    S.log("grip", f"夹爪 {a['action']}（缓慢移动中）")
                    settle_grip(arm)
                    st = arm.grip_state()
                    result = (f"夹爪 {a['action']} 完成；peak_tau={st.get('peak_tau')} "
                              f"grasped={st.get('grasped')} frozen={st.get('frozen')}")
                elif name == "look_gripper":
                    S.log("look", "查看夹爪特写")
                    result = "见下一轮的腕部图像"
                elif name == "reset":
                    arm.go_home(); settle(arm)
                    S.log("move", "回启动位（半速）")
                    result = "回启动位"
                elif name == "wait":
                    sec = float(np.clip(a.get("seconds", 2), 0.5, 15))
                    time.sleep(sec)
                    result = f"等待 {sec:.1f}s"
                else:
                    result = "未知工具"
                ok_step = not (isinstance(result, str) and
                               ("拒绝" in result or "没有航点" in result
                                or "未接管" in result or "失败" in result
                                or "有失败项" in result))
                S.log("step", f"第 {step_i}/{len(calls)} 步 {'✅ 成功' if ok_step else '❌ 失败'}："
                              f"{describe_call(name, a)}" +
                              ("" if ok_step else f"（{result[:60]}）"))
                messages.append({"role": "tool", "tool_call_id": call["id"], "content": result})
                if name in ("goto_waypoint", "goto_waypoints", "jog_multi", "reset"):
                    moved_any = ok_step      # 只有这一步成功，才计入"已动作"
                if name == "finish":
                    return
            # 动作执行完就本地判定：到位了就不再回头问模型
            if moved_any and allow_motion and not S.agent["done"]:
                if _motion_ok(arm):
                    dt = time.time() - t_start
                    S.log("finish", f"✅ GPT 控制成功：{task[:60]}")
                    print(f"[agent] 控制成功（{dt:.1f}s，误差 {arm.busy():.4f} rad）", flush=True)
                    S.agent["done"] = True
                    return
                S.log("warn", f"跟踪误差 {arm.busy():.3f} rad（> {SUCCESS_EPS}），继续下一轮")
            time.sleep(0.3)
        S.log("warn", "回合用尽")
    except Exception as exc:
        S.log("error", f"{type(exc).__name__}: {exc}")
        traceback.print_exc()
    finally:
        S.agent.update({"running": False, "done": True})


def play_sequence(name):
    with S.lock:
        seq = S.sequences.get(name)
    if not seq:
        return False
    if not getattr(S.arm, "engaged", False):
        S.log("error", "手臂未接管，无法执行轨迹")
        return False
    S.playing = True
    try:
        for wp_name in seq["waypoints"]:
            with S.lock:
                wp = S.waypoints.get(wp_name)
            if not wp:
                S.log("warn", f"轨迹 {name}: 缺少航点 {wp_name}")
                continue
            ok, why = S.arm.goto(wp["q"])
            if not ok:
                S.log("warn", f"[轨迹 {name}] {wp_name} 被拒绝：{why}")
                break
            S.arm.grip(wp["grip"])
            S.log("move", f"[轨迹 {name}] → {wp_name}")
            settle(S.arm)
            settle_grip(S.arm)
        S.log("finish", f"轨迹 {name} 执行完毕")
    finally:
        S.playing = False
    return True
