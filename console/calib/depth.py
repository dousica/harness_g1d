"""双目深度精度测试。

用标定板的已知角点做匹配点，比稠密立体匹配（SGBM）准得多：
  * 每个角点在左右目里都有唯一 ID → 对应关系是精确的
  * 88 个点可以统计出偏差和离散度
  * 同时用 PnP 的深度做独立参考（PnP 用板子的物理尺寸，与视差无关）

深度公式（校正后）：  Z = fx * B / d
理论不确定度：        σ_Z = Z² / (fx * B) * σ_d
"""
from __future__ import annotations

import cv2
import numpy as np

from board import BOARD, BoardSpec, board_object_points


def load_stereo(left_yaml, right_yaml, stereo_yaml):
    import yaml
    L = yaml.safe_load(open(left_yaml, encoding="utf-8"))
    R = yaml.safe_load(open(right_yaml, encoding="utf-8"))
    S = yaml.safe_load(open(stereo_yaml, encoding="utf-8"))
    K1 = np.array(L["camera_matrix"]["data"], float).reshape(3, 3)
    D1 = np.array(L["distortion_coefficients"]["data"], float).ravel()
    K2 = np.array(R["camera_matrix"]["data"], float).reshape(3, 3)
    D2 = np.array(R["distortion_coefficients"]["data"], float).ravel()
    size = (int(L["image_width"]), int(L["image_height"]))
    Rm = np.array(S["R"], float).reshape(3, 3)
    Tm = np.array(S["T"], float).ravel()
    return K1, D1, K2, D2, Rm, Tm, size


def depth_report(imgL, imgR, K1, D1, K2, D2, Rm, Tm, size,
                 cornersL, idsL, cornersR, idsR, spec: BoardSpec = BOARD):
    """给定左右目检测结果，算每个角点的深度并与 PnP 对比。"""
    if cornersL is None or cornersR is None:
        return {"ok": False, "why": "有一只眼睛没看到板子"}
    common = np.intersect1d(np.asarray(idsL).ravel(), np.asarray(idsR).ravel())
    if len(common) < 8:
        return {"ok": False, "why": f"共同角点太少（{len(common)}）"}

    # 立体校正
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, size, Rm, Tm, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    mapLx, mapLy = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_32FC1)
    mapRx, mapRy = cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_32FC1)

    iL = {int(v): i for i, v in enumerate(np.asarray(idsL).ravel())}
    iR = {int(v): i for i, v in enumerate(np.asarray(idsR).ravel())}
    pL = np.asarray(cornersL).reshape(-1, 2)[[iL[int(c)] for c in common]]
    pR = np.asarray(cornersR).reshape(-1, 2)[[iR[int(c)] for c in common]]

    # 把角点变换到校正坐标系
    def remap_points(pts, mx, my):
        out = []
        for x, y in pts:
            xi, yi = int(round(x)), int(round(y))
            if 0 <= yi < my.shape[0] and 0 <= xi < mx.shape[1]:
                out.append([mx[yi, xi], my[yi, xi]])
            else:
                out.append([x, y])
        return np.array(out, float)

    qL = remap_points(pL, mapLx, mapLy)
    qR = remap_points(pR, mapRx, mapRy)

    fx = float(P1[0, 0])
    B = float(-P2[0, 3] / P2[0, 0])          # 基线（米）
    d = qL[:, 0] - qR[:, 0]                   # 视差（像素）
    good = d > 1.0
    if good.sum() < 6:
        return {"ok": False, "why": f"有效视差太少（{int(good.sum())}）"}

    Z = fx * B / d[good]                      # 每个角点的深度

    # PnP 参考深度（用左目原始图 + 板子物理尺寸）
    obj = board_object_points(common).astype(np.float64)
    ok_pnp, rvec, tvec = cv2.solvePnP(obj, pL.astype(np.float64), K1, D1)
    Z_pnp = None
    if ok_pnp:
        Rm_ = cv2.Rodrigues(rvec)[0]
        pts_cam = (Rm_ @ obj.T).T + tvec.ravel()
        Z_pnp = pts_cam[:, 2]

    return {
        "ok": True,
        "points": int(good.sum()),
        "baseline_mm": round(B * 1000, 2),
        "fx_rect": round(fx, 2),
        "disparity_px": {"mean": round(float(d[good].mean()), 2),
                         "std": round(float(d[good].std()), 3)},
        "depth_mm": {"mean": round(float(Z.mean()) * 1000, 1),
                     "std": round(float(Z.std()) * 1000, 2),
                     "min": round(float(Z.min()) * 1000, 1),
                     "max": round(float(Z.max()) * 1000, 1)},
        "pnp_depth_mm": None if Z_pnp is None else {
            "mean": round(float(np.mean(Z_pnp)) * 1000, 1),
            "std": round(float(np.std(Z_pnp)) * 1000, 2)},
        "diff_vs_pnp_mm": None if Z_pnp is None else
            round(float((Z.mean() - np.mean(Z_pnp)) * 1000), 1),
        "theoretical_sigma_mm_per_px": round(float(Z.mean()**2 / (fx * B) * 1000), 2),
    }
