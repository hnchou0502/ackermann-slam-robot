#!/usr/bin/env python3
"""自動求兩張 PGM 佔據圖之間的剛體變換（旋轉＋平移），供 merge_maps_offline.py 合圖。

**演算法與防護全部委派給 `stm32_bridge.merge_math`**（2026-07-21 Codex R4-F1 重構：
本工具原本自帶一份搜尋邏輯，與節點的防護演進不同步——重複邏輯是漏洞溫床，砍掉）。
本檔只負責：PGM/yaml 讀取、像素→世界座標（含 PGM 的 y 翻轉）、報表輸出。

**防護（不過＝exit 3、不輸出 TF；明知風險要硬拿加 --force）**：
  - 飽和：相距很遠的角度同時近滿分＝假尖峰（CM1 實測教訓）
  - 歧義：次高峰分數比 > 0.80——含「不同角度」與「同角度不同平移」兩種
    （後者＝兩個相同房間/重複走廊的退化，R4-F1）
（--tol 已移除：膨脹評分被實驗證偽（假飽和），不再提供這個會射自己腳的選項。）

輸出 `TF tx ty yaw_rad`，慣例與 merge_maps_offline 一致：
  B 系點 (xb,yb) → A 系 = (tx + c·xb − s·yb, ty + s·xb + c·yb)

用法：python3 auto_align_maps.py <A前綴> <B前綴> [--coarse 度] [--json 輸出] [--force]
  前綴＝不含副檔名的路徑，需存在 <前綴>.pgm 與 <前綴>.yaml
實績：2026-07-20 客廳互補組 yaw+32.9°、5cm 吻合 80.3%（對照組 24.0%）。
"""
import sys
import os
import re
import math
import json

import numpy as np

try:
    from stm32_bridge import merge_math as mm
except ImportError:  # 裸跑（未 source workspace）時，從 repo 相對位置找
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "src", "stm32_bridge"))
    from stm32_bridge import merge_math as mm

OCC_THRESH = 100      # PGM 值 < 此值 ＝ 牆


def read_yaml(path):
    d = {}
    for line in open(path, encoding="utf-8"):
        m = re.match(r"^(\w+):\s*(.+)$", line.strip())
        if m:
            d[m.group(1)] = m.group(2)
    res = float(d["resolution"])
    origin = [float(x) for x in d["origin"].strip("[]").split(",")[:2]]
    return res, origin


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    i += 1
    w, h = int(tokens[1]), int(tokens[2])
    arr = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return w, h, arr


def wall_points(prefix):
    """牆格世界座標 (N,2)。⚠ PGM row 0 在影像頂端＝地圖 y 最大處，要翻轉
    （OccupancyGrid 不用翻——那是 merge_math.walls_from_grid 的事，兩者別搞混）。"""
    res, origin = read_yaml(prefix + ".yaml")
    w, h, arr = read_pgm(prefix + ".pgm")
    rows, cols = np.nonzero(arr < OCC_THRESH)
    x = origin[0] + (cols + 0.5) * res
    y = origin[1] + (h - 1 - rows + 0.5) * res
    return np.stack([x, y], axis=1), res


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) < 2:
        print(__doc__)
        return 1
    a_prefix, b_prefix = args[0], args[1]
    coarse_deg = (float(sys.argv[sys.argv.index("--coarse") + 1])
                  if "--coarse" in sys.argv else 1.0)
    json_out = (sys.argv[sys.argv.index("--json") + 1]
                if "--json" in sys.argv else None)
    force = "--force" in sys.argv

    a_pts, res_a = wall_points(a_prefix)
    b_pts, res_b = wall_points(b_prefix)
    if abs(res_a - res_b) > 1e-9:
        print(f"兩張圖解析度不同（{res_a} vs {res_b}），本工具不處理")
        return 1
    res = res_a
    print(f"A 牆格 {len(a_pts)} 個, B 牆格 {len(b_pts)} 個, 解析度 {res} m")

    out = mm.align(a_pts, b_pts, res, coarse_deg=coarse_deg)
    if out is None:
        print("搜尋失敗（B 沒有牆格？）")
        return 1

    sc, th, tx, ty = out["score"], out["yaw"], out["tx"], out["ty"]
    print(f"\n=== 最佳解 ===")
    print(f"yaw   {math.degrees(th):+.2f}°  ({th:+.6f} rad)")
    print(f"平移  tx={tx:+.3f} m  ty={ty:+.3f} m")
    print(f"分數  {sc*100:.1f}%  歧義比 {out['ambiguity']:.2f}"
          f"（{out['runner_kind']} 峰"
          + (f"，yaw {math.degrees(out['runner_yaw']):+.1f}°" if out["runner_yaw"] is not None else "")
          + f"）  飽和 {out['saturated']}")

    # 防護決策＝merge_math.accept（min_score=0：報表工具不以分數高低擋人，
    # 分數多低堪用由下游 merge_maps_offline 的 5cm 吻合率說話）
    ok, reason = mm.accept(out, conflict=0.0, min_score=0.0, max_conflict=1.0)
    if not ok:
        print(f"\n⚠ 防護攔下（{reason}）：此解不可信，已拒絕輸出 TF"
              f"（明知風險要硬拿，加 --force）")
        if not force:
            if json_out:
                json.dump({"ok": False, "reason": reason, **{k: out[k] for k in
                           ("score", "ambiguity", "saturated")}},
                          open(json_out, "w", encoding="utf-8"), indent=2)
            return 3

    print(f"\nTF {tx:.4f} {ty:.4f} {th:.6f}")
    print(f"\n接著跑：python3 merge_maps_offline.py {a_prefix} {b_prefix} <輸出目錄> "
          f"{tx:.4f} {ty:.4f} {th:.6f}")
    if json_out:
        json.dump({"ok": True, "tx": tx, "ty": ty, "yaw_rad": th,
                   "yaw_deg": math.degrees(th), "score": sc,
                   "ambiguity": out["ambiguity"], "runner_kind": out["runner_kind"],
                   "resolution": res, "a": a_prefix, "b": b_prefix},
                  open(json_out, "w", encoding="utf-8"), indent=2)
        print(f"（已寫入 {json_out}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
