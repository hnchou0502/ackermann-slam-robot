#!/usr/bin/env python3
"""用捲尺真值量地圖精度：把凍結的特徵端點吸附到牆格，比對地圖距離 vs 實測距離。

用途＝RM 系列（二遍續建）的驗收主工具。同一份特徵檔餵給「第一遍圖」與「二遍續建圖」，
兩張圖走完全相同的量測規則，差異才可比。

用法：
    python measure_ground_truth.py <features.json> <map.yaml> [--label 名稱] [--json out.json]

⚠ 兩個前提，違反了數字就沒有意義：
  1. **特徵檔必須在看到待測地圖之前就凍結**（進 git，時間戳可查）。事後增刪特徵、
     改端點、換統計方式＝自證作廢。
  2. **待測地圖必須與特徵檔同一個世界座標系**。RM 的二遍續建是載入第一遍 posegraph
     續建，座標系會延續，故成立；若換成獨立重建的新圖，座標系原點/朝向不同，
     近似座標會落到別的地方——本工具偵測到「最近牆格 > SNAP_WARN_CM」會警告，
     但不保證擋得住，換圖務必先確認座標系。
"""
import argparse
import json
import math
import sys

import numpy as np
from PIL import Image

# Windows 主控台預設 cp950，印中文會變亂碼/崩潰（本機已知坑）。
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

SNAP_WARN_CM = 10.0      # 吸附距離超過此值＝近似座標可能落錯地方
SPARSE_WALL_CELLS = 10   # 鄰域牆格少於此數＝端點定位不穩，標記出來
# 平手容差＝0.4 格（2cm）。依據：特徵端點的近似座標是從人工標記圖上讀出來的，讀圖誤差
# 本身約 ±2cm，故距離差小於 2cm 的候選格在物理上無法區分，必須一起納入。
# ⚠ 這個值不可為了讓數字好看而調——敏感度已實測（rm1_ground_truth.md §4）：
# tol 從 0.05 到 1.5 格，尺度偏差都在 -1.4%~-2.1%、MAE 都在 4.4~5.7cm，結論不變。
TIE_TOL_CELLS = 0.4

# ⚠ 為什麼不取單一最近格（2026-08-10 實測踩到）：
# 佔據格地圖上，一個牆角附近常有數個牆格與近似座標「距離完全相同」（實測 F4.P1 有三格
# 同為 3.5cm）。此時 argmin / argsort 等不同寫法會選到不同格，同一張圖同一組特徵
# 竟跑出不同數字——F4 的距離在 1.968~2.012 m 間跳，誤差 -2.2 / +0.1 / +2.2 cm 都可能，
# 連正負號都不確定。取單一格＝把「選格的任意性」偽裝成精確數字。
# 正確做法：把「最近距離 + TIE_TOL_CELLS 格」內的候選格全部列為端點候選，
# 對 P1×P2 所有組合算距離，回報中位數與範圍。範圍就是這條特徵的端點不確定度，
# 它必須明確出現在報告裡，否則後續「改善了幾公分」的比較是自欺。


def load_map(yaml_path):
    """讀 map_saver 產物（.yaml + .pgm），回傳 (occupied 布林陣列, res, origin)。"""
    meta = {}
    with open(yaml_path, encoding="utf-8") as fh:
        for line in fh:
            if ":" in line:
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(x) for x in meta["origin"].strip("[]").split(",")[:2]]

    img_name = meta["image"]
    img_path = yaml_path.rsplit("/", 1)[0] + "/" + img_name if "/" in yaml_path else img_name
    img_path = img_path.replace("\\", "/")
    try:
        grid = np.array(Image.open(img_path))
    except FileNotFoundError:
        # yaml 與 pgm 同目錄的一般情形；Windows 路徑分隔用 os.path 較穩
        import os
        grid = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), img_name)))
    return grid, res, (ox, oy)


def snap(occ_rows, occ_cols, h, res, origin, x, y, radius_m):
    """把世界座標 (x,y) 吸附到牆格。

    回傳 (候選座標 list, 最近距離cm, 鄰域牆格數)。候選＝距離在「最近 + TIE_TOL_CELLS 格」
    內的所有牆格——平手不可用任意規則打破，見檔頭說明。
    """
    ox, oy = origin
    r0 = h - (y - oy) / res
    c0 = (x - ox) / res
    d = np.hypot(occ_rows - r0, occ_cols - c0)
    near = d <= radius_m / res
    if not near.any():
        return None, None, 0
    dmin = float(d[near].min())
    sel = np.nonzero(near & (d <= dmin + TIE_TOL_CELLS))[0]
    cands = [(float(occ_cols[i] * res + ox), float((h - occ_rows[i]) * res + oy)) for i in sel]
    return cands, dmin * res * 100.0, int(near.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("features")
    ap.add_argument("map_yaml")
    ap.add_argument("--label", default="")
    ap.add_argument("--json", dest="json_out", default="")
    args = ap.parse_args()

    spec = json.load(open(args.features, encoding="utf-8"))
    radius = spec.get("snap_radius_m", 0.30)
    occ_thr = spec.get("occupied_below", 65)

    grid, res, origin = load_map(args.map_yaml)
    h = grid.shape[0]
    occ_rows, occ_cols = np.nonzero(grid < occ_thr)

    label = args.label or args.map_yaml
    print(f"地圖：{label}")
    print(f"  解析度 {res} m/格   原點 {origin}   尺寸 {grid.shape[1]}x{h} 格")
    print(f"  特徵檔 {args.features}（凍結於 {spec.get('frozen_at', '?')}）  吸附半徑 {radius} m\n")

    rows, warns = [], []
    for name, f in spec["features"].items():
        pts, snaps = [f["p1"], f["p2"]], []
        for tag, (x, y) in zip(("P1", "P2"), pts):
            cands, dist, n = snap(occ_rows, occ_cols, h, res, origin, x, y, radius)
            if cands is None:
                warns.append(f"{name}.{tag} 半徑 {radius}m 內沒有牆格——座標系可能不符")
                snaps.append(None)
                continue
            if dist > SNAP_WARN_CM:
                warns.append(f"{name}.{tag} 吸附距離 {dist:.1f}cm > {SNAP_WARN_CM}cm——近似座標可能落錯位置")
            if n < SPARSE_WALL_CELLS:
                warns.append(f"{name}.{tag} 鄰域僅 {n} 個牆格——端點定位不穩，該條誤差可信度較低")
            snaps.append((cands, dist, n))
        if any(s is None for s in snaps):
            continue
        ca, da, na = snaps[0]
        cb, db, nb = snaps[1]
        ds = sorted(math.hypot(bx - ax, by - ay) for ax, ay in ca for bx, by in cb)
        got = float(np.median(ds))
        unc = (ds[-1] - ds[0]) / 2.0          # 端點候選造成的不確定度（半區間）
        truth = f["truth_m"]
        rows.append({
            "name": name, "desc": f.get("desc", ""), "truth_m": truth,
            "map_m": got, "map_min_m": ds[0], "map_max_m": ds[-1], "unc_cm": unc * 100.0,
            "err_cm": (got - truth) * 100.0, "rel_pct": (got / truth - 1.0) * 100.0,
            "snap_cm": [round(da, 1), round(db, 1)], "wall_cells": [na, nb],
            "n_candidates": [len(ca), len(cb)],
        })

    print(f"{'特徵':<6}{'真值':>8}{'地圖(中位)':>11}{'不確定度':>10}{'誤差':>9}{'相對':>8}")
    for r in rows:
        print(f"{r['name']:<6}{r['truth_m']*100:>8.1f}{r['map_m']*100:>11.1f}"
              f"{'±'+format(r['unc_cm'], '.1f'):>10}{r['err_cm']:>+9.1f}{r['rel_pct']:>+7.1f}%")
    print("（單位 cm。不確定度＝端點在平手候選格間游移造成的距離變動，非量測噪聲）")

    if not rows:
        print("\n沒有任何特徵量測成功——請確認地圖座標系與特徵檔相符。")
        sys.exit(2)

    err = np.array([r["err_cm"] for r in rows]) / 100.0
    dist = np.array([r["truth_m"] for r in rows])
    unc = np.array([r["unc_cm"] for r in rows]) / 100.0
    mae = float(np.abs(err).mean()) * 100.0
    # MAE 的不確定度：各條端點不確定度獨立傳遞（保守，未含捲尺 ±1cm）
    mae_unc = float(np.sqrt((unc ** 2).sum()) / len(rows)) * 100.0
    # 過原點最小平方＝把誤差解釋成單一比例的尺度偏差
    k = float((err * dist).sum() / (dist * dist).sum())
    resid = err - k * dist
    resid_rms = float(np.sqrt((resid ** 2).mean())) * 100.0
    typ_unc = float(np.median(unc)) * 100.0

    print(f"\n主指標  平均絕對誤差 MAE = {mae:.2f} ± {mae_unc:.2f} cm   （{len(rows)} 條特徵）")
    print(f"診斷    最佳擬合尺度偏差 = {k*100:+.2f}%   扣除後殘差 RMS = {resid_rms:.2f} cm")
    print("        殘差：" + "  ".join(f"{r['name']}{v*100:+.1f}" for r, v in zip(rows, resid)))
    if resid_rms < typ_unc:
        print(f"        ⚠ 殘差 RMS 小於端點不確定度中位數（{typ_unc:.1f}cm）＝在本量測精度下"
              f"看不到尺度以外的誤差，不等於證明它不存在。")

    if warns:
        print("\n⚠ 警告：")
        for w in warns:
            print("  -", w)

    if args.json_out:
        out = {"label": label, "map_yaml": args.map_yaml, "features_file": args.features,
               "frozen_at": spec.get("frozen_at"), "rows": rows,
               "mae_cm": mae, "scale_bias_pct": k * 100.0, "residual_rms_cm": resid_rms,
               "warnings": warns}
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
        print(f"\n已寫出 {args.json_out}")


if __name__ == "__main__":
    main()
