#!/usr/bin/env python3
"""RM2／RM3 驗收條件②：第一遍已建區不得劣化。

    「限定在第一遍已知區範圍內比假牆格數與 occupied/free 衝突，二遍不得多於一遍
      （防『整體有一項改善但局部塗壞』鑽過驗收）。」

判定邏輯全在零 ROS 的 `stm32_bridge/map_compare_math.py`（筆電可 pytest）；
這支只負責讀檔與印報表。**不需要車或 ROS，純離線。**

用法：
    python3 compare_maps_masked.py --before <第一遍前綴> --after <第二遍前綴>

前綴＝不含副檔名，同名的 .pgm 與 .yaml 要在一起。PASS ⇒ exit 0。

⚠ 為什麼要容忍 1 格：兩張圖的 origin 不保證差整數格（RM2 實測差 13.7 格），
硬做逐格比對會把整條牆的邊緣算成衝突。嚴格逐格的數字也會一併印出來對照。
"""
import argparse
import sys

from PIL import Image

from stm32_bridge.map_compare_math import (
    DEFAULT_TOLERANCE_CELLS, cell_offset, inner_wall_cells, known_mask,
    masked_compare, verdict,
)

try:
    sys.stdout.reconfigure(encoding="utf-8")   # Windows 主控台 cp950 會爆中文
except Exception:
    pass


def load_map(prefix):
    """讀 .pgm + .yaml，回傳 (rows, res, origin, w, h)。"""
    im = Image.open(prefix + ".pgm").convert("L")
    w, h = im.size
    px = list(im.getdata())
    rows = [px[r * w:(r + 1) * w] for r in range(h)]
    res, origin = None, None
    with open(prefix + ".yaml") as f:
        for line in f:
            line = line.strip()
            if line.startswith("resolution:"):
                res = float(line.split(":", 1)[1])
            elif line.startswith("origin:"):
                nums = line.split("[", 1)[1].split("]")[0].split(",")
                origin = tuple(float(n) for n in nums[:2])
    if res is None or origin is None:
        raise SystemExit("讀不到 %s.yaml 的 resolution/origin" % prefix)
    return rows, res, origin, w, h


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--before", required=True, help="第一遍地圖前綴（不含副檔名）")
    ap.add_argument("--after", required=True, help="第二遍地圖前綴")
    ap.add_argument("--tolerance", type=int, default=DEFAULT_TOLERANCE_CELLS,
                    help="容忍幾格錯位（預設 %d；0＝嚴格逐格）" % DEFAULT_TOLERANCE_CELLS)
    a = ap.parse_args(argv)

    ar, ares, aorigin, aw, ah = load_map(a.before)
    br, bres, borigin, bw, bh = load_map(a.after)
    if abs(ares - bres) > 1e-9:
        raise SystemExit("兩張圖解析度不同（%.4f vs %.4f），本工具不做重取樣" % (ares, bres))

    drow, dcol, residual = cell_offset(ares, aorigin, ah, borigin, bh)

    print("第一遍：%s  %dx%d 格  origin %s" % (a.before.split("/")[-1], aw, ah, aorigin))
    print("第二遍：%s  %dx%d 格  origin %s" % (a.after.split("/")[-1], bw, bh, borigin))
    print("格子位移：drow=%d dcol=%d；次格殘差 %.2f 格%s"
          % (drow, dcol, residual,
             "（非整數格對齊 ⇒ 必須容忍，否則牆緣全算衝突）" if residual > 0.1 else ""))
    print()

    mask = known_mask(ar)
    before = inner_wall_cells(ar, mask=mask)
    # 第二遍的假牆也只算落在「第一遍已知區」對應位置的那些
    bmask = {(r + drow, c + dcol) for (r, c) in mask}
    after = inner_wall_cells(br, mask=bmask)

    cmp_tol = masked_compare(ar, br, drow, dcol, tolerance=a.tolerance)
    cmp_strict = masked_compare(ar, br, drow, dcol, tolerance=0)

    v = verdict(before, after, cmp_tol)

    print("比較範圍＝第一遍已知的 %d 格" % cmp_tol["masked_cells"])
    print("  一致                %6d (%.1f%%)"
          % (cmp_tol["agree"], 100.0 * cmp_tol["agree"] / max(1, cmp_tol["masked_cells"])))
    print("  牆→空地（畫錯）      %6d" % cmp_tol["occ_to_free"])
    print("  空地→牆（畫錯）      %6d" % cmp_tol["free_to_occ"])
    print("  已知→未知（資訊流失）%6d" % cmp_tol["known_to_unknown"])
    print("  〔嚴格逐格對照：牆→空地 %d／空地→牆 %d／已知→未知 %d〕"
          % (cmp_strict["occ_to_free"], cmp_strict["free_to_occ"],
             cmp_strict["known_to_unknown"]))
    print()
    print("第一遍已建區內的假牆：%d 格（%d 塊）→ %d 格（%d 塊）"
          % (before["inner_cells"], before["inner_components"],
             after["inner_cells"], after["inner_components"]))
    print()
    print("判決：%s" % ("PASS" if v["pass"] else "FAIL"))
    print("理由：%s" % v["reason"])
    return 0 if v["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
