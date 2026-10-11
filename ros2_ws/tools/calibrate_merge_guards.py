#!/usr/bin/env python3
"""合圖防護門檻的校準量測腳本（Codex R5-F2：校準必須可實跑重現，不是文件裡的描述）。

跑什麼：
  1. 兩組真實配對（客廳互補、房間全重疊）：align → free_conflict → accept
  2. 錯解負例：同兩組配對套恆等變換的空曠衝突
  3. 合成負例：直牆自對齊（真歧義）
輸出＝校準表（docs/design/merge_guard_calibration.md 的數字即本腳本輸出）。

用法（在 repo 的 ros2_ws/ 下）：
  PYTHONPATH=src/stm32_bridge python tools/calibrate_merge_guards.py ../data/maps
可選 --coarse <度>（預設 2.0，與 map_merge_live 預設一致）。
"""
import sys
import os
import math
import hashlib

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import auto_align_maps as aam            # PGM 讀取＋y 翻轉（也順便帶入 merge_math 路徑）
from stm32_bridge import merge_math as mm

PAIRS = {
    "客廳互補": ("c1_living1", "car2_living1"),
    "房間全重疊": ("c1_room2", "car2_room1"),
}


def free_points(prefix):
    """PGM 的「明確空曠」格（值 ≥250）世界座標——含 y 翻轉，與 wall_points 同慣例。"""
    res, org = aam.read_yaml(prefix + ".yaml")
    w, h, arr = aam.read_pgm(prefix + ".pgm")
    rows, cols = np.nonzero(arr >= 250)
    x = org[0] + (cols + 0.5) * res
    y = org[1] + (h - 1 - rows + 0.5) * res
    return np.stack([x, y], axis=1)


def sha16(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()[:16]


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    maps_dir = sys.argv[1]
    coarse = (float(sys.argv[sys.argv.index("--coarse") + 1])
              if "--coarse" in sys.argv else 2.0)

    print(f"maps_dir={maps_dir}  coarse_deg={coarse}  "
          f"SHIFT_SEP_M={mm.SHIFT_SEP_M}  門檻="
          f"{mm.DEFAULT_MIN_SCORE}/{mm.DEFAULT_MAX_AMBIGUITY}/{mm.DEFAULT_MAX_CONFLICT}")
    print("\n== 輸入雜湊（sha256[:16]）==")
    for _, (A, B) in PAIRS.items():
        for p in (A, B):
            for ext in (".pgm", ".yaml"):
                f = os.path.join(maps_dir, p + ext)
                print(f"  {p}{ext}: {sha16(f)}")

    print("\n== 真實配對（正解）與恆等變換（錯解負例）==")
    for name, (A, B) in PAIRS.items():
        pa, _ = aam.wall_points(os.path.join(maps_dir, A))
        pb, res = aam.wall_points(os.path.join(maps_dir, B))
        fa = free_points(os.path.join(maps_dir, A))
        out = mm.align(pa, pb, res, coarse_deg=coarse)
        conf = mm.free_conflict_ratio(fa, pb, out["tx"], out["ty"], out["yaw"], res)
        ok, reason = mm.accept(out, conf)
        conf_id = mm.free_conflict_ratio(fa, pb, 0.0, 0.0, 0.0, res)
        print(f"  {name}: yaw {math.degrees(out['yaw']):+.1f}° "
              f"t=({out['tx']:+.2f},{out['ty']:+.2f}) score {out['score']:.3f} "
              f"歧義 {out['ambiguity']:.3f}({out['runner_kind']}) 衝突 {conf:.3f} "
              f"→ accept={ok}({reason})；恆等衝突[錯解]={conf_id:.3f}")

    print("\n== 合成負例：直牆自對齊（真歧義）==")
    res = 0.05
    xs = np.arange(0.0, 3.0 + 1e-9, res)
    line = np.stack([xs, np.zeros_like(xs)], axis=1)
    out = mm.align(line, line.copy(), res, coarse_deg=coarse)
    ok, reason = mm.accept(out, 0.0)
    print(f"  直牆: score {out['score']:.3f} 歧義 {out['ambiguity']:.3f}"
          f"({out['runner_kind']}) → accept={ok}({reason})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
