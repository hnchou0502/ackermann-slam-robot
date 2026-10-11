"""兩張佔據地圖的「限定區域」比對（零 ROS 依賴，照 follow_math／merge_math 模式）。

RM2 驗收條件②（2026-08-30）。任務卡原文：

    「②**第一遍已建區不得劣化**——限定在第一遍已知區範圍內比假牆格數與 occupied/free
      衝突，二遍不得多於一遍（防『整體有一項改善但局部塗壞』鑽過驗收）。」

**為什麼要限定範圍**：第二遍會掃到第一遍沒去過的地方，整張圖的統計會被新區域稀釋。
把第二遍的新牆算進「劣化」是不公平的，把新區域的乾淨算進「改善」也是自欺。
唯一有意義的比較範圍＝**第一遍已經知道的那些格子**。

## 兩個實作上非做不可的處理

**一、次格錯位必須容忍。** 兩張圖的 origin 不保證差整數格：RM2 實測 y 原點差 0.685 m
＝ **13.7 格**。硬做逐格比對時，牆的邊緣會整條被判成衝突——那是對齊誤差，不是劣化。
故預設用 ±1 格容忍（與 RM1「5 cm 內吻合率」同一個尺度，地圖解析度就是 0.05 m）。
嚴格逐格的數字也一併回報，供判斷錯位有多嚴重。

**二、「變回未知」與「衝突」要分開算。** 第二遍把第一遍的牆改判成空地（或反過來）＝
真的畫錯了；第二遍把第一遍已知的格子變回未知＝資訊流失，性質不同、成因也不同
（多半是 lifelong 裁掉了對應的掃描）。混在一起會看不出是哪種問題。

地圖值慣例（nav2 map_saver trinary）：0＝障礙、254＝空地、205＝未知。
影像 row 0 在**上方**＝世界座標 y 最大處；world_y = origin_y + (height-1-row) * res。
"""

# --- 像素值語意（nav2 map_saver 的 trinary 輸出）---
OCC_MAX_V = 99      # <= 此值＝障礙（黑）
FREE_MIN_V = 206    # >= 此值＝空地（白 254）；介於兩者之間＝未知（灰 205）
# ⚠ 這個門檻必須 >205：未知格的值就是 205，設成 200 會把「未知」判成「空地」，
#   遮罩（＝第一遍已知區）就會涵蓋整張圖，判準②整個失去意義（2026-08-30 踩到）。
#   206 的由來＝yaml 的 free_thresh 0.196 換算：p=(255-v)/255 < 0.196 ⇒ v > 205.0。

OCC = "occ"
FREE = "free"
UNKNOWN = "unknown"

DEFAULT_TOLERANCE_CELLS = 1
"""比對時容忍幾格的錯位。

RM2 實測兩張圖的 y 原點相差 13.7 格（非整數），四捨五入後殘留 0.7 格誤差。
±1 格＝0.05 m，與 RM1 沿用的 5 cm 判準同尺度。設 0 ＝嚴格逐格。
"""

MIN_COMPONENT_CELLS = 3
"""小於這個格數的牆段當雜訊丟掉（沿用 map_components.py 的門檻）。"""


def classify(v):
    """把像素值轉成三態語意。"""
    if v <= OCC_MAX_V:
        return OCC
    if v >= FREE_MIN_V:
        return FREE
    return UNKNOWN


def to_world(row, col, res, origin, height):
    """影像格子 → 世界座標（格子中心）。row 0 在上方＝ y 最大。"""
    return (origin[0] + col * res, origin[1] + (height - 1 - row) * res)


def to_cell(x, y, res, origin, height):
    """世界座標 → 影像格子（四捨五入到最近格）。回傳 (row, col)。"""
    col = int(round((x - origin[0]) / res))
    row = int(round(height - 1 - (y - origin[1]) / res))
    return row, col


def cell_offset(res, origin_a, height_a, origin_b, height_b):
    """a 的格子 (row, col) 加上這個位移就是 b 的格子。

    回傳 (drow, dcol, residual_cells)——residual 是四捨五入丟掉的次格誤差，
    呼叫端應該據此決定容忍幾格。residual 若接近 0.5 表示兩張圖剛好錯半格，
    這時嚴格逐格比對必定產生大量假衝突。
    """
    dx = (origin_a[0] - origin_b[0]) / res
    dy = (origin_a[1] - origin_b[1]) / res
    dcol = int(round(dx))
    drow = int(round(-dy + (height_b - height_a)))
    res_col = abs(dx - round(dx))
    res_row = abs(dy - round(dy))
    return drow, dcol, max(res_col, res_row)


def _at(rows, r, c):
    if 0 <= r < len(rows) and 0 <= c < len(rows[0]):
        return classify(rows[r][c])
    return UNKNOWN


def _near(rows, r, c, want, tol):
    """(r, c) 周圍 tol 格內有沒有 want 這種格子。tol=0 就只看自己。"""
    for dr in range(-tol, tol + 1):
        for dc in range(-tol, tol + 1):
            if _at(rows, r + dr, c + dc) == want:
                return True
    return False


def masked_compare(a_rows, b_rows, drow, dcol, tolerance=DEFAULT_TOLERANCE_CELLS):
    """在「a 已知的格子」範圍內比較 a 與 b。

    參數
      a_rows/b_rows  二維像素值（列優先，row 0 在上）
      drow/dcol      a 的格子加上這個位移＝b 的格子（來自 cell_offset）
      tolerance      容忍幾格錯位

    回傳 dict：
      masked_cells      a 的已知格子數（比較範圍大小）
      occ_to_free       a 說是牆、b 說是空地（真的畫錯了）
      free_to_occ       a 說是空地、b 說是牆
      known_to_unknown  a 已知、b 變回未知（資訊流失，性質不同）
      agree             兩邊一致
    """
    out = {"masked_cells": 0, "occ_to_free": 0, "free_to_occ": 0,
           "known_to_unknown": 0, "agree": 0}
    for r, row in enumerate(a_rows):
        for c, v in enumerate(row):
            av = classify(v)
            if av == UNKNOWN:
                continue
            out["masked_cells"] += 1
            br, bc = r + drow, c + dcol
            bv = _at(b_rows, br, bc)
            if bv == av:
                out["agree"] += 1
                continue
            # 容忍錯位：附近有同類就算一致（次格對齊誤差不是劣化）
            if tolerance and _near(b_rows, br, bc, av, tolerance):
                out["agree"] += 1
                continue
            if bv == UNKNOWN:
                out["known_to_unknown"] += 1
            elif av == OCC:
                out["occ_to_free"] += 1
            else:
                out["free_to_occ"] += 1
    return out


def inner_wall_cells(rows, mask=None, min_size=MIN_COMPONENT_CELLS):
    """算「內部假牆」＝與影像邊界不相連的障礙連通塊的總格數。

    疊圖的本體就是這種浮在空地中間的牆段。與外牆相連的算真牆。
    mask 若給定（一組 (row, col)），只計算落在 mask 內的假牆格——
    這樣才能「限定在第一遍已知區範圍內」比較。

    ⚠ 連通性一律在**整張圖**上算，不能只在 mask 內算：把 mask 外的牆切掉會讓
    原本連到外牆的牆段變成「孤立」，憑空製造假牆。
    """
    h = len(rows)
    w = len(rows[0]) if h else 0
    occ = [[classify(rows[r][c]) == OCC for c in range(w)] for r in range(h)]
    seen = [[False] * w for _ in range(h)]
    total = 0
    comps = 0
    for r0 in range(h):
        for c0 in range(w):
            if not occ[r0][c0] or seen[r0][c0]:
                continue
            stack = [(r0, c0)]
            seen[r0][c0] = True
            cells = []
            touches_border = False
            while stack:
                r, c = stack.pop()
                cells.append((r, c))
                if r <= 1 or c <= 1 or r >= h - 2 or c >= w - 2:
                    touches_border = True
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < h and 0 <= nc < w and occ[nr][nc] and not seen[nr][nc]:
                            seen[nr][nc] = True
                            stack.append((nr, nc))
            if touches_border or len(cells) < min_size:
                continue
            inside = [x for x in cells if mask is None or x in mask]
            if inside:
                comps += 1
                total += len(inside)
    return {"inner_components": comps, "inner_cells": total}


def known_mask(rows):
    """回傳所有「已知」格子的座標集合（給 inner_wall_cells 當 mask）。"""
    return {(r, c) for r, row in enumerate(rows)
            for c, v in enumerate(row) if classify(v) != UNKNOWN}


def verdict(before, after, compare):
    """判準②：第一遍已建區不得劣化。

    參數
      before/after  inner_wall_cells 的結果（限定在同一個 mask 內）
      compare       masked_compare 的結果

    「不得多於一遍」是任務卡的原文，指假牆格數。衝突數一併回報供判讀。
    """
    worse = after["inner_cells"] > before["inner_cells"]
    return {
        "pass": not worse,
        "reason": ("第一遍已建區內的假牆從 %d 格變成 %d 格＝%s"
                   % (before["inner_cells"], after["inner_cells"],
                      "劣化" if worse else "沒有變多")),
        "inner_before": before["inner_cells"],
        "inner_after": after["inner_cells"],
        "conflicts": compare["occ_to_free"] + compare["free_to_occ"],
        "known_to_unknown": compare["known_to_unknown"],
    }
