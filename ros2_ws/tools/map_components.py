#!/usr/bin/env python3
"""數地圖裡「互不相連的牆段」有幾條——不受地圖大小影響的疊圖指標。

乾淨的房間：牆連成 1 條封閉輪廓（可能加少數幾件家具）。
疊圖的地圖：同一面牆被畫成多條錯位的線，內部浮出大量斷裂的假牆段。

只計 >=3 格的牆段（濾掉孤立雜點），並分別統計「碰到地圖邊界的」與「浮在內部的」。
用法：python3 map_components.py <pgm> [<pgm> ...]

⚠⚠ **僅適用於「空房間」，有家具的空間會嚴重誤判**（2026-07-20 客廳實測踩到）：
   沙發／桌子／椅腳本來就是與牆不相連的合法障礙物，會被算成「內部浮牆」。
   實例：car2 客廳圖回報 1009 格／30 段（與 X4 爛圖的 686 同級），但目視檢查
   牆全是單線、幾何一致，那 1009 格是家具（中央矩形＝桌／沙發，
   下方規律小點＝椅腳）。**數字必須配合目視判讀，不可單獨當結論**。
   空房間對照才是此指標成立的場景（bedroom：car1=3、car2=0、X4 map1=686）。
"""
import sys
from collections import deque
from PIL import Image

MIN_SIZE = 3


def load(path):
    im = Image.open(path).convert("L")
    w, h = im.size
    px = list(im.getdata())
    return w, h, [px[r * w:(r + 1) * w] for r in range(h)]


def analyse(path):
    w, h, g = load(path)
    occ = [[g[y][x] < 100 for x in range(w)] for y in range(h)]
    seen = [[False] * w for _ in range(h)]

    comps = []
    for y in range(h):
        for x in range(w):
            if not occ[y][x] or seen[y][x]:
                continue
            q = deque([(y, x)])
            seen[y][x] = True
            cells = []
            touches_border = False
            while q:
                cy, cx = q.popleft()
                cells.append((cy, cx))
                if cy <= 1 or cx <= 1 or cy >= h - 2 or cx >= w - 2:
                    touches_border = True
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w and occ[ny][nx] and not seen[ny][nx]:
                            seen[ny][nx] = True
                            q.append((ny, nx))
            if len(cells) >= MIN_SIZE:
                comps.append((len(cells), touches_border))

    if not comps:
        return None
    comps.sort(reverse=True)
    biggest = comps[0][0]
    total = sum(c for c, _ in comps)
    inner = [c for c, b in comps if not b]
    return dict(path=path.split("/")[-1], w=w, h=h,
                n=len(comps), biggest=biggest,
                biggest_pct=biggest / total * 100,
                inner_n=len(inner), inner_cells=sum(inner))


rows = [r for r in (analyse(p) for p in sys.argv[1:]) if r]

print(f"{'地圖':<14} {'尺寸':>8} {'牆段總數':>9} {'最大段佔比':>11} {'內部浮牆段':>11} {'內部浮牆格':>11}")
print("-" * 72)
for r in rows:
    print(f"{r['path']:<14} {r['w']}x{r['h']:<4} {r['n']:>9} "
          f"{r['biggest_pct']:>10.1f}% {r['inner_n']:>11} {r['inner_cells']:>11}")
print()
print("最大段佔比越高＝牆越連貫（理想接近 100%＝整圈牆連成一條）。")
print("內部浮牆段／格越多＝疊出來的假牆越多。")
