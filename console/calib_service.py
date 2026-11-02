"""标定服务：从现有相机流取帧、检测 ChArUco、评分、采集、解算。"""
from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

# calib/ 下的模块用裸名互相引用，这里把该目录加入搜索路径
sys.path.insert(0, str(Path(__file__).resolve().parent / "calib"))

import cv2
import numpy as np

import console_core as C
from board import BOARD, board_object_points, detect, make_detector
from intrinsics import IntrinsicSession
from stereo import StereoSession
from handeye import TcpKinematics, residuals, solve_eye_to_hand
from depth import depth_report, load_stereo
from quality import metrics as quality_metrics
from store import new_run_dir, write_intrinsics, write_json

CAMS = {"head_left", "head_right", "lwrist", "rwrist", "stereo"}
_SESSIONS: dict[str, IntrinsicSession] = {}
_AUTO: dict[str, dict] = {}
_REGIONS: dict[str, set] = {}
_HE: dict = {"samples": [], "dir": None, "kin": None, "K": None, "D": None}      # cam -> {on, stable_since, last_save, last_rvec}
_LOCK = threading.Lock()
_DETECTOR = None


def detector():
    global _DETECTOR
    if _DETECTOR is None:
        _DETECTOR = make_detector()
    return _DETECTOR


def frame_bgr(cam: str):
    """取指定相机的最新一帧（BGR）。head_left/right 从双目流里切。"""
    try:
        if cam in ("head_left", "head_right"):
            raw = C.eye_jpeg("left" if cam == "head_left" else "right")
        else:
            c = C.CAMS.get(cam)
            raw = c.jpeg() if c else None
    except Exception:
        raw = None
    if raw is None:
        return None
    return cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)


DATA_DIR = Path("/home/yushu/g1d_vision_data/calibration")


def _latest_dir(cam: str):
    """最近一个「真的有样本」的会话目录（按修改时间，不能按名字排序）。"""
    prefix = "head_stereo" if cam == "stereo" else cam
    dirs = []
    for d in DATA_DIR.glob(f"{prefix}_*"):
        if not d.is_dir():
            continue
        if not list(d.glob("sample_*.json")):
            continue
        dirs.append(d)
    dirs.sort(key=lambda d: d.stat().st_mtime, reverse=True)
    return dirs[0] if dirs else None


def _restore(cam: str):
    """从最近一次会话目录恢复样本（重启不丢）。"""
    d = _latest_dir(cam)
    if d is None:
        return None
    files = sorted(d.glob("sample_*.json"))
    if not files:
        return None
    s = StereoSession() if cam == "stereo" else IntrinsicSession(cam)
    for f in files:
        try:
            p = json.loads(f.read_text(encoding="utf-8"))
            obj = board_object_points(np.array(p["ids"]))
            size = tuple(p.get("image_size") or (1920, 1080))
            s.image_size = size
            if cam == "stereo":
                s.add_from_data(obj, p["corners_L"], p["corners_R"], f.stem)
            else:
                s.add_from_data(obj, p["corners"], f.stem)
        except Exception as exc:
            print(f'[calib] 恢复失败 {f.name}: {exc}', flush=True)
            continue
    s.run_dir = d
    print(f"[calib] 从 {d.name} 恢复 {len(s)} 个样本", flush=True)
    return s


def _write_sample(d, name, payload):
    try:
        write_json(d / f"{name}.json", payload)
    except Exception as exc:
        print(f"[calib] 写样本失败：{exc}", flush=True)


def session(cam: str):
    with _LOCK:
        if cam not in _SESSIONS:
            _SESSIONS[cam] = _restore(cam) or (
                StereoSession() if cam == "stereo" else IntrinsicSession(cam))
        return _SESSIONS[cam]


def score(cam: str) -> dict:
    img = frame_bgr("head_left" if cam == "stereo" else cam)
    if img is None:
        return {"ok": False, "corner_count": 0, "corner_ratio": 0.0, "area_ratio": 0.0,
                "sharpness": 0.0, "reasons": ["相机没有画面"], "samples": len(session(cam))}
    corners, ids = detect(img, detector())
    m = quality_metrics(img, corners, ids, BOARD.total_corners)
    m["samples"] = len(session(cam))
    m["auto"] = bool(auto_state(cam).get("on"))
    m["regions"] = regions(cam)
    m["image_size"] = [int(img.shape[1]), int(img.shape[0])]
    return m


def annotate(img, corners, ids, m: dict):
    out = img.copy()
    if corners is not None and len(corners):
        cv2.aruco.drawDetectedCornersCharuco(out, corners, ids, (0, 255, 0))
    color = (0, 200, 0) if m.get("ok") else (0, 0, 255)
    text = f"{m.get('corner_count',0)} pts  {m.get('area_ratio',0)*100:.0f}%  " \
           f"sharp {m.get('sharpness',0):.0f}"
    cv2.putText(out, text, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    if not m.get("ok"):
        why = " / ".join(m.get("reasons", []))[:60]
        cv2.putText(out, why, (14, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return out


def set_auto(cam: str, on: bool):
    _AUTO[cam] = {"on": bool(on), "stable_since": None, "last_save": 0.0, "last_rvec": None}
    return _AUTO[cam]


def auto_state(cam: str) -> dict:
    return _AUTO.get(cam) or {"on": False}


def region_of(corners, image_shape) -> int:
    """角点质心落在画面 3x3 的哪一格（0..8）。"""
    h, w = image_shape[:2]
    pts = np.asarray(corners).reshape(-1, 2)
    cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
    col = min(2, max(0, int(cx / (w / 3.0))))
    row = min(2, max(0, int(cy / (h / 3.0))))
    return row * 3 + col


def regions(cam: str):
    return sorted(_REGIONS.get(cam, set()))


def _mark_region(cam: str, corners, shape):
    _REGIONS.setdefault(cam, set()).add(region_of(corners, shape))


def _rvec_for(img, corners, ids):
    """给稳定性判断用的粗略板位姿。"""
    import cv2
    from board import board_object_points
    h, w = img.shape[:2]
    K = np.array([[0.9 * max(w, h), 0, w / 2], [0, 0.9 * max(w, h), h / 2], [0, 0, 1]], float)
    obj = board_object_points(ids).astype(np.float64)
    imgp = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    ok, rvec, _ = cv2.solvePnP(obj, imgp, K, np.zeros(5))
    return rvec if ok else None


def _auto_tick(cam: str, img, corners, ids, m):
    """自动采集：稳定 + 合格 + 位姿有差异 → 存一张。"""
    import cv2
    st = _AUTO.get(cam)
    if not st or not st["on"]:
        return
    now = time.monotonic()
    if not m.get("ok") or corners is None:
        st["stable_since"] = None
        return
    if now - st["last_save"] < 0.8:      # 冷却
        return
    rvec = _rvec_for(img, corners, ids)
    if rvec is None:
        return
    if st["last_rvec"] is not None:
        R1, _ = cv2.Rodrigues(rvec)
        R0, _ = cv2.Rodrigues(st["last_rvec"])
        cos = (np.trace(R1 @ R0.T) - 1.0) / 2.0
        if np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))) > 1.2:
            st["stable_since"] = None    # 还在动
            st["last_rvec"] = rvec
            return
    else:
        st["last_rvec"] = rvec
        st["stable_since"] = now
        return
    if st["stable_since"] is None:
        st["stable_since"] = now
        return
    if now - st["stable_since"] < 0.30:  # 要稳 0.3 秒
        return
    ok, why, n = capture(cam)
    st["last_save"] = now
    st["stable_since"] = None
    if ok:
        print(f"[calib] 自动采集 {cam} 第 {n} 张", flush=True)


def stream_frame(cam: str, width=1280):
    """给浏览器的一帧：检测 + 标注 + 缩放，返回 JPEG 字节。"""
    img = frame_bgr("head_left" if cam == "stereo" else cam)
    if img is None:
        return None
    corners, ids = detect(img, detector())
    m = quality_metrics(img, corners, ids, BOARD.total_corners)
    try:
        _auto_tick(cam, img, corners, ids, m)
    except Exception as exc:
        print(f"[calib] 自动采集异常：{exc}", flush=True)
    out = annotate(img, corners, ids, m)
    h, w = out.shape[:2]
    out = cv2.resize(out, (width, int(width * h / w)))
    ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 78])
    return buf.tobytes() if ok else None


def capture_stereo():
    s = session("stereo")
    imgL = frame_bgr("head_left")
    imgR = frame_bgr("head_right")
    if imgL is None or imgR is None:
        return False, "双目没有画面", len(s)
    cL, idL = detect(imgL, detector())
    cR, idR = detect(imgR, detector())
    mL = quality_metrics(imgL, cL, idL, BOARD.total_corners)
    mR = quality_metrics(imgR, cR, idR, BOARD.total_corners)
    if not (mL["ok"] and mR["ok"]):
        return False, "左目：" + "；".join(mL["reasons"]) + " 右目：" + "；".join(mR["reasons"]), len(s)
    name = f"pair_{len(s):03d}"
    ok, why = s.add(imgL, cL, idL, imgR, cR, idR, name)
    if not ok:
        return False, why, len(s)
    d = getattr(s, "run_dir", None)
    if d is None:
        d = new_run_dir("calibration", "head_stereo")
        s.run_dir = d
    _REGIONS.setdefault("stereo", set()).add(region_of(cL, imgL.shape))
    cv2.imwrite(str(d / f"{name}_L.png"), annotate(imgL, cL, idL, mL))
    cv2.imwrite(str(d / f"{name}_R.png"), annotate(imgR, cR, idR, mR))
    common = np.intersect1d(np.asarray(idL).ravel(), np.asarray(idR).ravel())
    iL = {int(v): i for i, v in enumerate(np.asarray(idL).ravel())}
    iR = {int(v): i for i, v in enumerate(np.asarray(idR).ravel())}
    _write_sample(d, "sample_" + name, {
        "ids": [int(v) for v in common],
        "corners_L": np.asarray(cL).reshape(-1, 2)[[iL[int(c)] for c in common]].tolist(),
        "corners_R": np.asarray(cR).reshape(-1, 2)[[iR[int(c)] for c in common]].tolist(),
        "image_size": [int(imgL.shape[1]), int(imgL.shape[0])]})
    return True, "ok", len(s)


def capture(cam: str):
    if cam == "stereo":
        return capture_stereo()
    img = frame_bgr(cam)
    if img is None:
        return False, "相机没有画面", 0
    corners, ids = detect(img, detector())
    m = quality_metrics(img, corners, ids, BOARD.total_corners)
    if not m["ok"]:
        return False, "；".join(m["reasons"]) or "质量不合格", len(session(cam))
    name = f"{len(session(cam)):03d}.png"
    ok, why = session(cam).add(img, corners, ids, name)
    if not ok:
        return False, why, len(session(cam))
    _mark_region(cam, corners, img.shape)
    # 一个会话只建一次目录，所有图片放同一个文件夹
    d = getattr(session(cam), "run_dir", None)
    if d is None:
        d = new_run_dir("calibration", cam)
        session(cam).run_dir = d
    cv2.imwrite(str(d / name), annotate(img, corners, ids, m))
    _write_sample(d, "sample_" + name.replace(".png", ""), {
        "ids": [int(v) for v in np.asarray(ids).ravel()],
        "corners": np.asarray(corners).reshape(-1, 2).tolist(),
        "image": name, "camera": cam,
        "image_size": [int(img.shape[1]), int(img.shape[0])]})
    write_json(d / "session.json", {"camera": cam, "board": BOARD.__dict__,
                                    "samples": len(session(cam))})
    # 记住本次会话目录
    session(cam).run_dir = d
    return True, "ok", len(session(cam))


def run_stereo():
    s = session("stereo")
    try:
        res = s.calibrate()
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    d = getattr(s, "run_dir", None) or new_run_dir("calibration", "head_stereo")
    path_l = write_intrinsics(d / "head_left.yaml", "head_left", res["K_left"], res["D_left"],
                              res["image_size"])
    path_r = write_intrinsics(d / "head_right.yaml", "head_right", res["K_right"],
                              res["D_right"], res["image_size"])
    import yaml
    stereo = {
        "image_width": res["image_size"][0], "image_height": res["image_size"][1],
        "rms_stereo": res["rms_stereo"], "baseline_m": res["baseline_m"],
        "R": res["R"], "T": res["T"],
        "left": str(path_l), "right": str(path_r),
    }
    (d / "head_stereo.yaml").write_text(
        yaml.safe_dump(stereo, sort_keys=False, allow_unicode=True), encoding="utf-8")
    write_json(d / "head_stereo_calibration.json", res)
    return {"ok": True, "rms": res["rms_stereo"], "samples": res["pairs"],
            "baseline_m": res["baseline_m"], "K": res["K_left"],
            "path": str(d / "head_stereo.yaml")}


def run(cam: str):
    if cam == "stereo":
        return run_stereo()
    s = session(cam)
    try:
        res = s.calibrate()
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    d = getattr(s, "run_dir", None) or new_run_dir("calibration", cam)
    path = write_intrinsics(d / f"{cam}.yaml", cam, res["K"], res["D"], res["image_size"])
    write_json(d / f"{cam}_calibration.json", res)
    return {"ok": True, "rms": res["rms"], "samples": res["samples"],
            "K": res["K"], "D": res["D"], "path": str(path)}


# ---------------------------------------------------------------- 手眼标定
def _latest_intrinsics(cam="head_left"):
    import yaml
    dirs = sorted([d for d in DATA_DIR.glob("head_stereo_*") if (d / f"{cam}.yaml").exists()],
                  key=lambda d: d.stat().st_mtime, reverse=True)
    if not dirs:
        return None
    d = yaml.safe_load((dirs[0] / f"{cam}.yaml").read_text(encoding="utf-8"))
    K = np.array(d["camera_matrix"]["data"], float).reshape(3, 3)
    D = np.array(d["distortion_coefficients"]["data"], float).ravel()
    return K, D


def _ensure_kin():
    if _HE["kin"] is None:
        mcfg = C.S.arm.cfg["model"] if isinstance(C.S.arm.cfg, dict) else C.S.arm.cfg.model
        dirs = list(mcfg.get("package_dirs") or []) or ["/home/yushu/g1d_ros2_lzh/src"]
        _HE["kin"] = TcpKinematics(mcfg["urdf"], dirs)
    if _HE["K"] is None:
        got = _latest_intrinsics("head_left")
        if got is None:
            raise RuntimeError("找不到 head_left 内参，请先做双目标定")
        _HE["K"], _HE["D"] = got
    return _HE["kin"]


def handeye_state():
    return {"samples": len(_HE["samples"]),
            "has_intrinsics": _HE["K"] is not None or _latest_intrinsics() is not None,
            "dir": str(_HE["dir"]) if _HE["dir"] else None}


def handeye_capture(hand="left"):
    """记录一个手眼样本：当前末端位姿 + 板子在头部相机里的位姿。"""
    import cv2
    arm = C.S.arm
    # 手眼采集只需要手臂【静止】，不要求我们接管（FK 用的是实测角度）
    q_a = arm.q()
    time.sleep(0.35)
    q_b = arm.q()
    if float(np.max(np.abs(np.asarray(q_b) - np.asarray(q_a)))) > 0.003:
        return {"ok": False, "why": "手臂还在动，请等它停稳"}
    try:
        kin = _ensure_kin()
    except Exception as exc:
        return {"ok": False, "why": str(exc)}
    img = frame_bgr("head_left")
    if img is None:
        return {"ok": False, "why": "头部相机没有画面"}
    corners, ids = detect(img, detector())
    if corners is None or len(ids) < 20:
        return {"ok": False, "why": f"板子角点太少（{0 if ids is None else len(ids)}）"}
    obj = board_object_points(ids).astype(np.float64)
    imgp = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, imgp, _HE["K"], _HE["D"])
    if not ok:
        return {"ok": False, "why": "板子位姿解算失败"}
    proj, _ = cv2.projectPoints(obj, rvec, tvec, _HE["K"], _HE["D"])
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - imgp) ** 2, axis=1))))
    if rms > 2.0:
        return {"ok": False, "why": f"板子位姿残差 {rms:.1f} px 过大"}
    C_cam_board = np.eye(4)
    C_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
    C_cam_board[:3, 3] = tvec.ravel()
    q = arm.q()
    tcp = f"{hand}_dex1_tcp_link"
    G = kin.torso_T_tcp(q, tcp)
    if _HE["dir"] is None:
        _HE["dir"] = new_run_dir("handeye", f"head_{hand}")
    name = f"sample_{len(_HE['samples']):03d}"
    _HE["samples"].append({"name": name, "G": G.tolist(), "C": C_cam_board.tolist(),
                           "q": np.asarray(q).tolist(), "rms_px": rms})
    write_json(_HE["dir"] / f"{name}.json",
               {"G": G.tolist(), "C": C_cam_board.tolist(), "q": np.asarray(q).tolist(),
                "rms_px": rms, "tcp": tcp})
    return {"ok": True, "count": len(_HE["samples"]), "rms": rms}


def handeye_solve(base="torso_link", tcp="left_dex1_tcp_link",
                  camera="head_left_camera_optical_frame"):
    if len(_HE["samples"]) < 3:
        return {"ok": False, "error": f"样本太少：{len(_HE['samples'])}"}
    G = [np.array(x["G"]) for x in _HE["samples"]]
    Cs = [np.array(x["C"]) for x in _HE["samples"]]
    try:
        X, Y, info = solve_eye_to_hand(G, Cs)
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    res = residuals(G, Cs, X, Y)
    if _HE["dir"] is None:
        _HE["dir"] = new_run_dir("handeye", "head_left")
    import yaml
    payload = {
        "kind": "eye_to_hand",
        "method": "Park-Martin rotation-log SVD + translation LS",
        "base_frame": base, "camera_frame": camera, "tcp_frame": tcp, "units": "meters",
        "samples": len(G), "pairs_used": info["pairs"],
        "translation_rms_mm": res["translation_rms_mm"],
        "rotation_rms_deg": res["rotation_rms_deg"],
        "T_torso_camera": X.tolist(),
    }
    path = _HE["dir"] / f"handeye_result_{time.strftime('%Y%m%d_%H%M%S')}.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return {"ok": True, "rms_mm": res["translation_rms_mm"], "rms_deg": res["rotation_rms_deg"],
            "samples": len(G), "pairs": info["pairs"],
            "T": np.round(X, 4).tolist(), "path": str(path)}


def handeye_reset():
    _HE["samples"] = []
    _HE["dir"] = None
    return {"ok": True}


def depth_live():
    """用最新一帧左右目做深度精度测试（不动机器人）。"""
    d = _latest_dir("stereo")
    if d is None:
        return {"ok": False, "why": "找不到双目标定目录"}
    need = ["head_left.yaml", "head_right.yaml", "head_stereo.yaml"]
    if not all((d / n).exists() for n in need):
        return {"ok": False, "why": f"{d.name} 里缺少 {need}"}
    imgL = frame_bgr("head_left")
    imgR = frame_bgr("head_right")
    if imgL is None or imgR is None:
        return {"ok": False, "why": "双目没有画面"}
    cL, idL = detect(imgL, detector())
    cR, idR = detect(imgR, detector())
    if cL is None or cR is None:
        return {"ok": False, "why": "有一只眼睛没检测到板子"}
    K1, D1, K2, D2, Rm, Tm, size = load_stereo(
        d / "head_left.yaml", d / "head_right.yaml", d / "head_stereo.yaml")
    out = depth_report(imgL, imgR, K1, D1, K2, D2, Rm, Tm, size, cL, idL, cR, idR)
    out["from"] = d.name
    return out


def reset(cam: str):
    with _LOCK:
        _SESSIONS[cam] = IntrinsicSession(cam)
    return True
