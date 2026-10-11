#!/usr/bin/env python3
"""拿既有地圖離線預演自主探索：車站在某點時，explorer 找不找得到目標？

動機（2026-08-11，EX3 架空輪測試時發現）：explorer 在小空間會判定「所有邊界都到不了」
直接收工——目標點要退離邊界 0.5 m、且周圍 0.25 m 全淨空，空間一小就全部不合格。
EX4 是實地驗收（要清場、人全程在旁），開跑才發現車不動代價太高，先用既有客廳圖預演。

做法：
  1. 讀 map_saver 存的 pgm/yaml（完整掃過的圖）
  2. **用射線投射模擬光達視線**，重建「車剛起步時只看得到附近」的部分地圖
     （不用「半徑內全部已知」的簡化——那會讓牆後面也變已知，抽不到真實的邊界形狀）
  3. 跑真正的 explore_math：抽邊界 → 叢集 → 挑目標
  4. 掃描 retreat／clearance 參數，看哪一組在這個場地找得到目標

用法：
    PYTHONPATH=../src/stm32_bridge python3 explore_dryrun.py <map.yaml> [起點x 起點y]
    （起點省略＝用地圖中心附近第一個空地格）
"""
import math
import os
import sys

from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'stm32_bridge'))

from stm32_bridge.explore_math import (  # noqa: E402
    Blacklist, Grid, choose_goal, extract_frontiers, goal_for_frontier,
    goal_made_progress,
    DEFAULT_CLEARANCE_M, DEFAULT_MIN_FRONTIER_CELLS, DEFAULT_RETREAT_M,
)

LIDAR_RANGE_M = 16.0        # C1 實測量程
LIDAR_BEAMS = 720           # C1 實測每圈點數


def load_map(yaml_path):
    """讀 map_saver 的 pgm+yaml → (完整 Grid, 起點資訊)。

    ⚠ 兩個容易出錯的地方：
      ① PGM 第一列是地圖「最上面」（y 最大），OccupancyGrid 的 row 0 是 y 最小 → 必須翻轉
      ② 像素語意用 map_saver 的**寫入**慣例：254=空地、0=障礙、205=未探索。
         不要照 map_server 的讀取門檻算（(255-205)/255=0.196 < free_thresh 0.25，
         會把「未探索」讀成「空地」，邊界就整個消失了）
    """
    meta = {}
    with open(yaml_path, encoding='utf-8') as f:
        for line in f:
            if ':' in line and not line.strip().startswith('#'):
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    res = float(meta['resolution'])
    origin = [float(x) for x in meta['origin'].strip('[]').split(',')[:2]]
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])

    im = Image.open(pgm).convert('L')
    w, h = im.size
    px = list(im.getdata())

    data = []
    for j in range(h):                       # j=0 是 y 最小 → 取 PGM 的最後一列
        row = px[(h - 1 - j) * w:(h - j) * w]
        for p in row:
            data.append(100 if p < 100 else (0 if p > 240 else -1))
    return Grid(data, w, h, res, origin[0], origin[1])


def simulate_scan(full, sx, sy, rng=LIDAR_RANGE_M, beams=LIDAR_BEAMS):
    """從 (sx,sy) 射線投射，回傳「只看得到的部分」的新 Grid（其餘標未探索）。"""
    out = [-1] * (full.width * full.height)
    step = full.resolution * 0.5
    for b in range(beams):
        a = 2 * math.pi * b / beams
        ca, sa = math.cos(a), math.sin(a)
        d = 0.0
        while d <= rng:
            x, y = sx + ca * d, sy + sa * d
            i, j = full.cell_of(x, y)
            if not (0 <= i < full.width and 0 <= j < full.height):
                break
            idx = j * full.width + i
            v = full.data[idx]
            if v >= 65:                      # 打到牆：標成障礙，射線終止
                out[idx] = 100
                break
            out[idx] = 0 if v == 0 else -1   # 空地標空地；原本就未知的維持未知
            d += step
    return Grid(out, full.width, full.height, full.resolution,
                full.origin_x, full.origin_y)


def pick_start(full):
    """挑一個在空地上、且四周夠開闊的起點（模擬車擺在客廳中間）。"""
    best = None
    for j in range(2, full.height - 2, 2):
        for i in range(2, full.width - 2, 2):
            if not full.is_free(i, j):
                continue
            x, y = full.world_of(i, j)
            if full.clear_around(x, y, 0.5):
                # 取離地圖中心最近的合格點
                cx = full.origin_x + full.width * full.resolution / 2
                cy = full.origin_y + full.height * full.resolution / 2
                d = math.hypot(x - cx, y - cy)
                if best is None or d < best[0]:
                    best = (d, x, y)
    return (best[1], best[2]) if best else (None, None)


def report(grid, robot, retreat, clearance, min_cells=DEFAULT_MIN_FRONTIER_CELLS):
    fs = extract_frontiers(grid, min_cells=min_cells)
    ok = sum(1 for f in fs if goal_for_frontier(grid, f, retreat, clearance) is not None)
    goal, _ = choose_goal(grid, fs, robot, None, retreat, clearance)
    return len(fs), ok, goal


def merge(acc, new):
    """把新一輪掃描併進累積地圖（已知的不會變回未知）。"""
    for k, v in enumerate(new.data):
        if v >= 0:
            acc.data[k] = v
    return acc


def multi_step(full, sx, sy, retreat, clearance, max_steps=30):
    """模擬車一站一站走：掃描→挑目標→移動到目標→再掃描，直到沒有目標可去。

    ⚠ 簡化：車直接「瞬移」到目標，不模擬 Nav2 的路徑與可達性。
      所以步數是**樂觀下限**，真實會更多（要繞路、會有規劃失敗進黑名單）。
      這個模擬要回答的是「探索會不會收斂」與「覆蓋率能到多少」，不是精確工時。
    """
    acc = simulate_scan(full, sx, sy)
    total_known = sum(1 for v in full.data if v >= 0)
    bl = Blacklist()                      # 與真實 explorer 同一套：沒進展的目標會被封鎖
    x, y = sx, sy
    path = 0.0
    rows = []
    for step in range(1, max_steps + 1):
        fs = extract_frontiers(acc)
        goal, _ = choose_goal(acc, fs, (x, y), bl, retreat, clearance)
        known = sum(1 for v in acc.data if v >= 0)
        rows.append((step - 1, known, 100.0 * known / total_known, path,
                     len(fs), len(bl), goal))
        if goal is None:
            break
        path += math.hypot(goal[0] - x, goal[1] - y)
        x, y = goal[0], goal[1]
        acc = merge(acc, simulate_scan(full, x, y))
        after = sum(1 for v in acc.data if v >= 0)
        if not goal_made_progress(known, after):
            bl.add(goal[0], goal[1])      # 去了沒收穫 → 封鎖，否則會無限迴圈
    return rows


def grid_starts(full, spacing=1.0):
    """在空地上每隔 spacing 公尺取一個合法起點（確定性，不用隨機種子）。"""
    out = []
    stride = max(1, int(round(spacing / full.resolution)))
    for j in range(0, full.height, stride):
        for i in range(0, full.width, stride):
            if not full.is_free(i, j):
                continue
            x, y = full.world_of(i, j)
            if full.no_obstacle_around(x, y, 0.25):
                out.append((x, y))
    return out


def sweep(full, retreat, clearance, spacing=1.0):
    """多起點掃描：探索結果對起點極度敏感，單一起點的數字沒有代表性。

    2026-08-11 實測：起點差 1 mm（同一格內），覆蓋率 81.4% vs 42.7%。
    這是「貪婪挑最近目標＋格子離散化」的本質——微小差異改變排序，
    之後整條路徑發散。所以要報分布，不是報單一數字。
    """
    starts = grid_starts(full, spacing)
    total = sum(1 for v in full.data if v >= 0)
    res = []
    for (x, y) in starts:
        vis = sum(1 for v in simulate_scan(full, x, y).data if v >= 0)
        rows = multi_step(full, x, y, retreat, clearance)
        last = rows[-1]
        # 覆蓋率, 步數, 路徑, 起點x, 起點y, 起步可見格數
        res.append((last[2], last[0], last[3], x, y, vis))
    return res


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    full = load_map(sys.argv[1])
    known = sum(1 for v in full.data if v >= 0)
    print('完整地圖：%dx%d 格 = %.2f x %.2f m，已知 %d 格'
          % (full.width, full.height, full.width * full.resolution,
             full.height * full.resolution, known))

    if '--sweep' in sys.argv:
        k = sys.argv.index('--sweep')
        spacing = float(sys.argv[k + 1]) if len(sys.argv) > k + 1 else 1.0
        print()
        print('--- 多起點掃描（每隔 %.2f m 取一個合法起點）---' % spacing)
        res = sweep(full, DEFAULT_RETREAT_M, DEFAULT_CLEARANCE_M, spacing)
        cov = sorted(r[0] for r in res)
        n = len(cov)
        print('起點數 %d' % n)
        print('  覆蓋率：最低 %.1f%% / 25%% %.1f%% / 中位 %.1f%% / 75%% %.1f%% / 最高 %.1f%%'
              % (cov[0], cov[n // 4], cov[n // 2], cov[3 * n // 4], cov[-1]))
        ge70 = sum(1 for c in cov if c >= 70.0)
        print('  達到 EX4 門檻(>=70%%) 的起點：%d/%d = %.0f%%' % (ge70, n, 100.0 * ge70 / n))
        print()
        print('  逐點明細（依起步可見範圍排序）：')
        print('    %-12s %-9s %-6s %-8s %s' % ('起步可見(m2)', '最終覆蓋', '步數', '路徑', '起點'))
        for c, st, pa, x, y, vis in sorted(res, key=lambda r: r[5]):
            area = vis * full.resolution ** 2
            flag = '  <-- 未達 70%' if c < 70 else ''
            print('    %-12.1f %-8.1f%% %-6d %-7.1fm (%6.2f,%6.2f)%s'
                  % (area, c, st, pa, x, y, flag))
        print()
        # 起步可見範圍與成敗的關係＝SOP「車該擺哪裡」的判準
        ok = [r[5] * full.resolution ** 2 for r in res if r[0] >= 70]
        ng = [r[5] * full.resolution ** 2 for r in res if r[0] < 70]
        if ok and ng:
            print('  達標起點的起步可見面積：%.1f ~ %.1f m2（中位 %.1f）'
                  % (min(ok), max(ok), sorted(ok)[len(ok) // 2]))
            print('  未達標起點的起步可見面積：%.1f ~ %.1f m2（中位 %.1f）'
                  % (min(ng), max(ng), sorted(ng)[len(ng) // 2]))
        return 0

    if len(sys.argv) >= 4:
        sx, sy = float(sys.argv[2]), float(sys.argv[3])
    else:
        sx, sy = pick_start(full)
        if sx is None:
            print('找不到夠開闊的起點')
            return 1
    si, sj = full.cell_of(sx, sy)
    state = ('空地' if full.is_free(si, sj) else
             '障礙' if full.is_occupied(si, sj) else '未探索')
    print('模擬起點：(%.4f, %.4f) → 格(%d,%d) 狀態=%s' % (sx, sy, si, sj, state))
    if not full.is_free(si, sj):
        print('⚠ 起點不在空地上——射線投射會從牆裡/未知區開始，結果無意義。')
        print('  真實的車不可能擺在那裡，換一個起點。')
        return 1
    if not full.no_obstacle_around(sx, sy, 0.25):
        print('⚠ 起點周圍 0.25 m 內有障礙——車放不下，結果不代表真實情境。')
        return 1

    part = simulate_scan(full, sx, sy)
    pk = sum(1 for v in part.data if v >= 0)
    print('車剛起步看到的部分地圖：已知 %d 格（佔完整圖的 %.0f%%）' % (pk, 100.0 * pk / known))
    print()

    print('--- 起步當下（部分地圖）---')
    print('%-10s %-10s %-8s %-8s %s' % ('retreat', 'clearance', '邊界叢集', '可用目標', '選中的目標'))
    for retreat, clearance in ((DEFAULT_RETREAT_M, DEFAULT_CLEARANCE_M),
                               (0.4, 0.2), (0.3, 0.2), (0.3, 0.15), (0.2, 0.15)):
        n, ok, goal = report(part, (sx, sy), retreat, clearance)
        g = '(%.2f, %.2f)' % (goal[0], goal[1]) if goal else '**找不到**'
        mark = '  <- 現行預設' if (retreat, clearance) == (DEFAULT_RETREAT_M, DEFAULT_CLEARANCE_M) else ''
        print('%-10.2f %-10.2f %-8d %-8d %s%s' % (retreat, clearance, n, ok, g, mark))

    print()
    print('--- 對照：完整掃完的地圖（殘留的未探索區＝家具後方/角落）---')
    n, ok, goal = report(full, (sx, sy), DEFAULT_RETREAT_M, DEFAULT_CLEARANCE_M)
    print('邊界叢集 %d 個、可用目標 %d 個 → %s'
          % (n, ok, '會正確收工' if ok == 0 else '仍有地方可去'))

    print()
    print('--- 多步模擬：車一站一站走，用現行預設參數 ---')
    rows = multi_step(full, sx, sy, DEFAULT_RETREAT_M, DEFAULT_CLEARANCE_M)
    print('%-5s %-9s %-9s %-9s %-7s %s'
          % ('步', '已知格數', '覆蓋率', '累計路徑', '黑名單', '下一個目標'))
    for step, known, pct, path, nfs, nbl, goal in rows:
        g = '(%.2f, %.2f)' % (goal[0], goal[1]) if goal else '無 → EXPLORATION DONE'
        print('%-5d %-9d %-8.1f%% %-8.2fm %-7d %s' % (step, known, pct, path, nbl, g))
    last = rows[-1]
    print()
    print('結論：%d 步後收工，覆蓋 %.1f%%，累計路徑 %.1f m'
          % (last[0], last[2], last[3]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
