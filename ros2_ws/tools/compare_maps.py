#!/usr/bin/env python3
"""比對兩台車對同一房間建的圖，量出里程尺度差異。

方法：取「牆格的外接框」當房間量測值。牆是光達從遠處掃到的，不必開到牆邊也量得到，
所以外接框不受「哪台車走得比較遠」影響——受影響的是里程尺度（走 1m 以為走了 1.1m
的車，會把房間畫大 10%）。

⚠ 不可改用「空曠格外接框」：那個會被探索覆蓋率影響（沒開到的角落是未知不是空曠）。
"""
import sys
from PIL import Image

# Windows 主控台預設 cp950，印中文會變亂碼/崩潰（本機已知坑）。
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

RES = 0.05


def wall_bbox(path):
    im = Image.open(path).convert("L")
    w, h = im.size
    px = im.load()
    xs, ys = [], []
    for j in range(h):
        for i in range(w):
            if px[i, j] < 100:      # 牆
                xs.append(i)
                ys.append(j)
    if not xs:
        return None
    return dict(path=path.split("\\")[-1].split("/")[-1],
                grid=f"{w}x{h}",
                wall_n=len(xs),
                width=(max(xs) - min(xs) + 1) * RES,
                height=(max(ys) - min(ys) + 1) * RES)


rows = [wall_bbox(p) for p in sys.argv[1:]]
rows = [r for r in rows if r]

print(f"{'地圖':<18} {'格網':>9} {'牆格':>6} {'房間寬':>9} {'房間高':>9}")
print("-" * 58)
for r in rows:
    print(f"{r['path']:<18} {r['grid']:>9} {r['wall_n']:>6} "
          f"{r['width']:>8.2f}m {r['height']:>8.2f}m")

if len(rows) == 2:
    a, b = rows
    dw = (b["width"] - a["width"]) / a["width"] * 100
    dh = (b["height"] - a["height"]) / a["height"] * 100
    print()
    print(f"以 {a['path']} 為基準：")
    print(f"  寬度差 {b['width']-a['width']:+.3f} m  ({dw:+.1f}%)")
    print(f"  高度差 {b['height']-a['height']:+.3f} m  ({dh:+.1f}%)")
    avg = (abs(dw) + abs(dh)) / 2
    print()
    if avg < 3:
        print(f"→ 平均差異 {avg:.1f}%，在量測噪聲內（格子解析度 5cm，本身就是 ±2.5% 級）")
        print("  car2 里程尺度無明顯偏差，**不需要校正**")
    elif avg < 8:
        print(f"→ 平均差異 {avg:.1f}%，略高於噪聲，可列入觀察但非急迫")
    else:
        print(f"→ 平均差異 {avg:.1f}%，明顯偏差，建議做里程 scale 標定"
              f"（工具 ros2_ws/tools/calibrate_odom_scale.py）")
