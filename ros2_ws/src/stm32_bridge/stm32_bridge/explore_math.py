"""自主探索的純數學與狀態機（零 ROS 依賴，照 follow_math／merge_math 模式；Windows 直接 pytest）。

EX2（2026-08-11）。內容分四塊，全部不碰 ROS，才能在筆電上把邊界情境測完：

  1. 邊界抽取／叢集／挑目標   —— 車下一步該去哪
  2. 黑名單                   —— 去不了的地方別一直去
  3. MapHealth（地圖過期狀態機）—— 地圖不再更新時必須停手，不能繼續照舊圖派目標
  4. GoalLifecycle（目標生命週期）—— 同一時刻至多一個有效目標，違反就是雙重控制

**為什麼 3 和 4 要寫成狀態機而不是幾個 if**：它們是安全設計的一部分。
本批任務的安全主不變量是「任何必要元件異常退出 ⇒ launch 全關 ⇒ serial 斷 ⇒ 韌體 500ms
watchdog 停車」。explorer 遇到「取消目標卻得不到確認」這種狀況時，**正確反應是自殺**
（讓主不變量接手停車），而不是「不再送新目標、等 Nav2 自己停」——後者沒有任何機制
保證那個還活著的目標會終止，車可能繼續開。這條寫死在 GoalLifecycle.tick()。

座標慣例：世界座標 (x, y) 單位公尺，與 OccupancyGrid 的 origin 對齊；
格子索引 (i, j)＝(column, row)，row 0 在 origin 側（與 merge_math.walls_from_grid 同慣例）。
"""
import math
from collections import deque

# --- 佔據網格的值語意（slam_toolbox 輸出 0／100／-1，門檻沿用 nav2 慣例）---
UNKNOWN = -1
FREE_MAX = 25            # <= 此值視為空地（nav2 free_thresh 0.25）
OCC_MIN = 65             # >= 此值視為障礙（nav2 occupied_thresh 0.65）

# --- 政策常數（測試鎖住這些值；改動要附實測理由）---
DEFAULT_MIN_FRONTIER_CELLS = 6      # 小於此格數的邊界叢集丟掉＝雜訊或掃描縫隙，
                                    # 6 格 × 0.05 m = 0.3 m，約等於車身寬 0.191 m 再加餘裕
DEFAULT_RETREAT_M = 0.5             # 目標點自邊界往已知區退這麼多（阿克曼可達性，packet A4）
MIN_RETREAT_M = 0.30                # 退避的硬下限：車頭比 base_link 前伸 0.28 m，
                                    # 退得比這更少＝車停好時車頭已伸進未探索區。
                                    # 角落起點退不進 0.5 m 時才會用到（見 goal_for_frontier）
DEFAULT_CLEARANCE_M = 0.25          # 目標點周圍這個半徑內不得有障礙
DEFAULT_BLACKLIST_RADIUS_M = 0.5    # 失敗目標的封鎖半徑（≥車身長 0.337 m）
DEFAULT_PREEMPT_RATIO = 0.5         # 新目標近到「不足現有目標距離的一半」才值得換
DEFAULT_MAP_STALE_TIMEOUT_S = 10.0  # 地圖多久沒更新算過期
MAX_FUTURE_STAMP_S = 1.0            # 時戳超前現在超過這麼多＝非法幀
DEFAULT_CANCEL_TIMEOUT_S = 3.0      # 取消目標多久沒被確認＝致命（見檔頭）


class Grid:
    """佔據網格的輕量容器（不依賴 nav_msgs，測試可直接手刻）。"""

    def __init__(self, data, width, height, resolution, origin_x=0.0, origin_y=0.0):
        if len(data) != width * height:
            raise ValueError('data 長度 %d 與 %dx%d 不符' % (len(data), width, height))
        self.data = data
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

    def at(self, i, j):
        """越界一律回 UNKNOWN——地圖邊界外本來就是沒探索過的地方。"""
        if not (0 <= i < self.width and 0 <= j < self.height):
            return UNKNOWN
        return self.data[j * self.width + i]

    def is_free(self, i, j):
        v = self.at(i, j)
        return 0 <= v <= FREE_MAX

    def is_occupied(self, i, j):
        return self.at(i, j) >= OCC_MIN

    def is_unknown(self, i, j):
        return self.at(i, j) == UNKNOWN

    def world_of(self, i, j):
        """格子索引 → 格心的世界座標。"""
        return (self.origin_x + (i + 0.5) * self.resolution,
                self.origin_y + (j + 0.5) * self.resolution)

    def cell_of(self, x, y):
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def clear_around(self, x, y, radius):
        """範圍內「全部已知且無障礙」（未知也算不通過）。

        ⚠ **不要拿這個當探索目標的合格條件**——2026-08-11 離線預演實測，
        用它會讓探索走 1 步就收工（覆蓋率卡在 38%）。理由見 no_obstacle_around。
        這個嚴格版留給「需要確認腳下完全已知」的場合（例如挑模擬起點）。
        """
        n = max(1, int(math.ceil(radius / self.resolution)))
        i0, j0 = self.cell_of(x, y)
        for dj in range(-n, n + 1):
            for di in range(-n, n + 1):
                if not self.is_free(i0 + di, j0 + dj):
                    return False
        return True

    def no_obstacle_around(self, x, y, radius):
        """範圍內沒有障礙即可，**未知格可以接受**。

        探索目標要用這個判定。原因（2026-08-11 用 c1_living1 離線預演抓到）：
        邊界的定義就是「已知空地與未知區的交界」，它旁邊必然有未知區。
        若要求周圍全部已知，等於自相矛盾——只有深入已知區的點才合格，
        而那些點走過去也掃不到新東西，於是探索走一步就宣告完成（覆蓋率 38%，
        遠低於 EX4 要求的 70%）。

        「車不會開進沒掃過的地方」由 Nav2 的 `allow_unknown: false` 保證（規劃層），
        不該在挑目標這一層重複把關到動彈不得。
        """
        n = max(1, int(math.ceil(radius / self.resolution)))
        i0, j0 = self.cell_of(x, y)
        for dj in range(-n, n + 1):
            for di in range(-n, n + 1):
                if self.is_occupied(i0 + di, j0 + dj):
                    return False
        return True


# ---------------------------------------------------------------------------
# 1. 邊界抽取／叢集／挑目標
# ---------------------------------------------------------------------------

class Frontier:
    """一叢相連的邊界格（已知空地與未知區的交界）。"""

    def __init__(self, cells, grid):
        self.cells = cells
        self.grid = grid
        xs = [grid.world_of(i, j)[0] for (i, j) in cells]
        ys = [grid.world_of(i, j)[1] for (i, j) in cells]
        self.centroid = (sum(xs) / len(cells), sum(ys) / len(cells))
        self.size = len(cells)

    def __repr__(self):
        return 'Frontier(size=%d, centroid=(%.2f, %.2f))' % (
            self.size, self.centroid[0], self.centroid[1])


def find_frontier_cells(grid):
    """回傳所有邊界格：本身是空地、且四鄰至少有一格未知。

    只認 4-鄰域（不含斜角）：斜角相鄰在格子世界裡是「穿過角落」，
    實際上車過不去，算進來會生出貼著牆角的假邊界。
    """
    out = []
    for j in range(grid.height):
        for i in range(grid.width):
            if not grid.is_free(i, j):
                continue
            if (grid.is_unknown(i + 1, j) or grid.is_unknown(i - 1, j)
                    or grid.is_unknown(i, j + 1) or grid.is_unknown(i, j - 1)):
                out.append((i, j))
    return out


def cluster_cells(cells):
    """把邊界格分成一叢一叢（8-連通；叢集內部允許斜角相連，不影響可達性判斷）。"""
    remaining = set(cells)
    clusters = []
    while remaining:
        seed = remaining.pop()
        group = [seed]
        q = deque([seed])
        while q:
            i, j = q.popleft()
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    nb = (i + di, j + dj)
                    if nb in remaining:
                        remaining.discard(nb)
                        group.append(nb)
                        q.append(nb)
        clusters.append(group)
    return clusters


def extract_frontiers(grid, min_cells=DEFAULT_MIN_FRONTIER_CELLS):
    """完整流程：抽邊界格 → 叢集 → 丟掉太小的。回傳 Frontier 清單（大的在前）。"""
    clusters = cluster_cells(find_frontier_cells(grid))
    fs = [Frontier(c, grid) for c in clusters if len(c) >= min_cells]
    fs.sort(key=lambda f: -f.size)
    return fs


def _free_side_normal(grid, frontier):
    """邊界叢集的「已知空地那一側」的單位法向量。

    做法＝對叢集內每格，累加指向鄰域空地格的單位向量再平均。
    退避方向必須用這個算，不能用「質心指向車子」——車可能繞了半圈停在別的方位，
    那樣退避會把目標推去奇怪的地方（甚至推進牆裡）。
    """
    ax = ay = 0.0
    for (i, j) in frontier.cells:
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                if grid.is_free(i + di, j + dj):
                    n = math.hypot(di, dj)
                    ax += di / n
                    ay += dj / n
    m = math.hypot(ax, ay)
    if m < 1e-9:
        return None
    return (ax / m, ay / m)


def goal_for_frontier(grid, frontier, retreat=DEFAULT_RETREAT_M,
                      clearance=DEFAULT_CLEARANCE_M):
    """把一個邊界叢集換成可下達的目標 (x, y, yaw)；不可用時回 None。

    - 位置＝叢集質心往已知空地側退 retreat 公尺（阿克曼車無法貼著未知區急停轉向）
    - 朝向＝從目標點看向叢集質心，讓車面對未知區，掃描面直接吃進新區域
    - 目標點本身必須是**已知空地**（Nav2 才規劃得到；allow_unknown 是 false）
    - 周圍 clearance 內不得有**障礙**；有未知區沒關係——邊界旁邊本來就有未知區，
      要求周圍全已知會讓探索一步就停（見 Grid.no_obstacle_around 的實測紀錄）
    """
    n = _free_side_normal(grid, frontier)
    if n is None:
        return None
    cx, cy = frontier.centroid
    step = grid.resolution * 2

    # 先試指定的退避距離，往內多退幾步；整條都不行才改「退少一點」再試。
    # 為什麼需要這個備援（2026-08-11 多起點掃描實測）：退避寫死 0.5 m 時，
    # 車擺在角落／貼牆處會**一個目標都生不出來**——18 個起點裡有 2 個 0 步就收工、
    # 覆蓋率 5–6%。退避備援讓那些位置也找得到目標。
    # ⚠ **下限 MIN_RETREAT_M 不可再降**：車頭比定位中心前伸 0.28 m，
    #   退得比這更少＝車停好時車頭已經伸進沒掃過的區域。
    for r in (retreat, max(MIN_RETREAT_M, retreat * 0.7), MIN_RETREAT_M):
        d = r
        for _ in range(6):                  # 每個退避距離再往內退 6 步
            gx, gy = cx + n[0] * d, cy + n[1] * d
            gi, gj = grid.cell_of(gx, gy)
            if grid.is_free(gi, gj) and grid.no_obstacle_around(gx, gy, clearance):
                yaw = math.atan2(cy - gy, cx - gx)
                return (gx, gy, yaw)
            d += step
    return None


def choose_goal(grid, frontiers, robot_xy, blacklist=None,
                retreat=DEFAULT_RETREAT_M, clearance=DEFAULT_CLEARANCE_M):
    """挑下一個目標：能用的之中離車最近的。回傳 (goal, frontier) 或 (None, None)。

    最近優先＝經典 explore_lite 策略。不用「最大叢集優先」是因為小空間裡
    大叢集往往在房間另一頭，來回跑的時間成本高於多掃幾次。
    """
    best = None
    for f in frontiers:
        g = goal_for_frontier(grid, f, retreat, clearance)
        if g is None:
            continue
        if blacklist is not None and blacklist.contains(g[0], g[1]):
            continue
        d = math.hypot(g[0] - robot_xy[0], g[1] - robot_xy[1])
        if best is None or d < best[0]:
            best = (d, g, f)
    if best is None:
        return (None, None)
    return (best[1], best[2])


# --- choose_goal 回傳空之後，這代表什麼？（2026-08-30 EX4 實車三趟後補上）---
NO_FRONTIER = "no_frontier"
"""連一個邊界叢集都沒有了＝真的探索完了。"""

ALL_BLACKLISTED = "all_blacklisted"
"""每個能產生目標的邊界都已經試過並失敗＝該收工了。"""

NOT_READY = "not_ready"
"""邊界還在，但**現在**產不出合法目標——多半是已知區太小、退不出立足點。
**這不是探索完成**，是還不能動；地圖長大之後就可能可以了。"""


def classify_no_goal(grid, frontiers, blacklist=None,
                     retreat=DEFAULT_RETREAT_M, clearance=DEFAULT_CLEARANCE_M):
    """`choose_goal` 回傳 None 時，判斷這是「探索完成」還是「還不能動」。

    **為什麼需要這個函式**：2026-08-30 EX4 實車三趟，explorer 三趟都提早收工。
    原因是它把 `choose_goal` 回傳 None 一律當成「探索完成」，但那個 None 其實混了
    三種完全不同的狀況。run3 的實測最能說明問題：

        EXPLORATION DONE：no reachable frontier left (7 clusters, 3 blacklisted)

    7 個叢集還在、只有 3 個試過失敗，**剩下 4 個連試都沒試**就宣告完成，
    覆蓋率只有 39%（判準要 ≥70%）。run2 更極端：車還沒動、地圖 98% 未知、
    4 個叢集全部產不出目標，23 秒就收工。

    回傳 NO_FRONTIER／ALL_BLACKLISTED／NOT_READY 其中之一。
    **前兩者才可以收工**，NOT_READY 必須繼續等（總時限是保險）。

    ⚠ 混合情況（有些邊界產不出目標、有些產得出但被黑名單）一律回 NOT_READY：
      產不出目標的那些**可能隨地圖長大而變得可用**，此時宣告完成就是重演這個 bug。
      代價是可能等到總時限才結束——但「提早收工」比「等久一點」危險得多：
      前者會讓一趟驗收直接作廢，後者只是慢。
    """
    if not frontiers:
        return NO_FRONTIER

    formable = 0
    for f in frontiers:
        g = goal_for_frontier(grid, f, retreat, clearance)
        if g is None:
            # 有邊界產不出目標 ⇒ 地圖再長大就可能可以，不准當成探索完成
            return NOT_READY
        formable += 1

    # 每個邊界都產得出目標，卻走到這裡 ⇒ 全被黑名單擋下（試過且失敗）
    return ALL_BLACKLISTED if formable else NOT_READY


def should_preempt(current_goal, current_frontier_centroid, grid, frontiers,
                   robot_xy, blacklist=None, ratio=DEFAULT_PREEMPT_RATIO,
                   match_radius=None):
    """要不要放棄現在的目標改去別處。回傳 (要不要, 原因字串)。

    兩種情況才換（換目標要先取消、有成本，不能因為排序抖動就換來換去）：
      1. 現在的目標所屬叢集不見了（那塊已經被掃掉）→ 再開過去是白跑
      2. 出現明顯更近的目標：新距離 < 現在距離 × ratio
    """
    if match_radius is None:
        match_radius = max(4 * grid.resolution, DEFAULT_RETREAT_M)

    still_there = any(
        math.hypot(f.centroid[0] - current_frontier_centroid[0],
                   f.centroid[1] - current_frontier_centroid[1]) <= match_radius
        for f in frontiers)
    if not still_there:
        return (True, 'target frontier gone')

    g, _ = choose_goal(grid, frontiers, robot_xy, blacklist)
    if g is None:
        return (False, '')
    d_new = math.hypot(g[0] - robot_xy[0], g[1] - robot_xy[1])
    d_cur = math.hypot(current_goal[0] - robot_xy[0], current_goal[1] - robot_xy[1])
    if d_new < d_cur * ratio:
        return (True, 'much closer frontier found (%.2f < %.2f x %.2f)'
                % (d_new, d_cur, ratio))
    return (False, '')


def goal_made_progress(known_before, known_after, min_gain=1):
    """去了那個目標之後，地圖有沒有長出新東西？

    **沒有進展的目標必須列入黑名單，否則會無限迴圈**（2026-08-11 離線預演抓到）：
    目標只在「失敗」時進黑名單，但「成功抵達卻什麼都沒掃到」完全沒被處理——
    車抵達 → 邊界還在 → 又選同一個目標 → 立刻抵達 → …
    模擬中從第 10 步起卡住，覆蓋率停在 55.4% 直到步數上限；真實系統會空轉到探索時限用完。
    """
    return (known_after - known_before) >= min_gain


GOAL_STATUS_SUCCEEDED = 4     # action_msgs/GoalStatus
GOAL_STATUS_CANCELED = 5

ARRIVED = 'arrived'
ARRIVED_NO_GAIN = 'arrived_no_gain'
PREEMPTED = 'preempted'
PREEMPTED_NO_GAIN = 'preempted_no_gain'
FAILED = 'failed'


def judge_goal_result(status, preempted_by_us, known_before, known_after):
    """目標結束後要不要把它列黑名單。回傳 (要不要, 分類)。

    - 抵達：地圖有長大＝好目標；沒長大＝列黑名單（防無限迴圈，見 goal_made_progress）
    - **我們自己換目標而取消**（should_preempt）：**不是目標的錯**，比照抵達——
      地圖有長大就不列。2026-09-24 run5：5 次主動換目標（4 次「目標邊界已被掃掉」、
      1 次「有更近的」）全被當成失敗列入黑名單，每次封掉半徑 0.5 m。
      地圖沒長大仍然列，保留「兩個目標之間來回換」的防呆。
    - 其他（被中止、被拒、非我們要求的取消、狀態不明）＝失敗，列黑名單。

    known_before 為 None（送出時還沒有地圖）時視為「有收穫」，與既有抵達分支一致。
    """
    no_gain = known_before is not None and not goal_made_progress(known_before, known_after)
    if status == GOAL_STATUS_SUCCEEDED:
        return (no_gain, ARRIVED_NO_GAIN if no_gain else ARRIVED)
    if status == GOAL_STATUS_CANCELED and preempted_by_us:
        return (no_gain, PREEMPTED_NO_GAIN if no_gain else PREEMPTED)
    return (True, FAILED)


# ---------------------------------------------------------------------------
# 2. 黑名單
# ---------------------------------------------------------------------------

class Blacklist:
    """去不了的目標點。TTL＝整場（探索一趟通常十分鐘內，過期重試沒有意義，
    反而會讓車在同一個到不了的角落反覆卡住）。"""

    def __init__(self, radius=DEFAULT_BLACKLIST_RADIUS_M):
        self.radius = radius
        self.points = []

    def add(self, x, y):
        self.points.append((x, y))

    def contains(self, x, y):
        return any(math.hypot(x - px, y - py) <= self.radius
                   for (px, py) in self.points)

    def __len__(self):
        return len(self.points)


# ---------------------------------------------------------------------------
# 3. 地圖過期狀態機
# ---------------------------------------------------------------------------

class MapHealth:
    """判斷「地圖還在更新嗎」，過期就要求停手。

    狀態：ACTIVE（可派目標）／PAUSED（停派新目標並取消現有目標）。

    非法時戳一律當成「這幀不算數」，不是當成新資料：
      - NaN／inf／零值：來源沒設好時戳，拿來算年齡會得到荒謬結果
      - 超前現在 > MAX_FUTURE_STAMP_S：時鐘不同步，會讓地圖永遠「很新」而遮蔽真正的過期
      - 時戳沒有前進（重複舊幀）：不重置計時，否則一個卡住的節點反覆重發同一幀
        就能永遠騙過過期偵測
      - now 倒退（/clock 被重置）：立刻 PAUSED，因為所有年齡計算的基準都不可信了
    """

    ACTIVE = 'ACTIVE'
    PAUSED = 'PAUSED'

    def __init__(self, timeout=DEFAULT_MAP_STALE_TIMEOUT_S,
                 max_future=MAX_FUTURE_STAMP_S, recover_frames=2):
        self.timeout = timeout
        self.max_future = max_future
        self.recover_frames = recover_frames
        self.state = self.ACTIVE
        self.reason = ''
        self.last_stamp = None          # 最後一幀「合法且前進」的時戳
        self.last_now = None            # 上次餵進來的現在時刻（偵測時鐘倒退）
        self.paused_at_now = None       # 進入 PAUSED 的 **ROS 時刻**（不是地圖時戳）
        self._good_since_pause = 0

    # -- 內部 --
    def _stamp_ok(self, stamp, now):
        if stamp is None:
            return False, 'stamp is None'
        try:
            if math.isnan(stamp) or math.isinf(stamp):
                return False, 'stamp not finite'
        except TypeError:
            return False, 'stamp not a number'
        if stamp == 0:
            return False, 'stamp is zero'
        if stamp - now > self.max_future:
            return False, 'stamp %.3f is %.3fs in the future' % (stamp, stamp - now)
        return True, ''

    def _check_clock(self, now):
        if self.last_now is not None and now < self.last_now:
            self._pause('ros time went backwards (%.3f < %.3f)' % (now, self.last_now), now)
            self.last_now = now
            return True
        self.last_now = now
        return False

    def _pause(self, reason, now):
        if self.state != self.PAUSED:
            # 記 ROS 時刻而非「最後地圖時戳」：後者會讓「遞增但仍然很舊」的幀
            # 通過恢復判定（Codex F2，2026-08-11）。
            self.paused_at_now = now
            self._good_since_pause = 0
        self.state = self.PAUSED
        self.reason = reason

    # -- 對外 --
    def on_map(self, stamp, now):
        """收到一幀地圖。回傳 (狀態, 原因)。"""
        if self._check_clock(now):
            return (self.state, self.reason)

        ok, why = self._stamp_ok(stamp, now)
        if not ok:
            # 非法幀＝這幀不算數，年齡照舊算下去（可能因此過期）
            self.reason = 'illegal stamp: ' + why
            return self.tick(now, _keep_reason=True)

        advanced = self.last_stamp is None or stamp > self.last_stamp
        if not advanced:
            # 重複／倒退的舊幀不重置計時
            self.reason = 'duplicate or older map stamp'
            return self.tick(now, _keep_reason=True)

        self.last_stamp = stamp
        if self.state == self.PAUSED:
            # 恢復條件（Codex F2）：合法＋單調前進（上面已保證）＋**晚於進入 PAUSED 的
            # ROS 時刻**＋**相對 now 未過期**。少了最後一項，一張舊了 98 秒的地圖只要
            # 連送兩筆遞增時戳就能解除暫停，車會照陳舊地圖繼續派目標。
            fresh_enough = (now - stamp) <= self.timeout
            after_pause = self.paused_at_now is None or stamp > self.paused_at_now
            if fresh_enough and after_pause:
                self._good_since_pause += 1
                if self._good_since_pause >= self.recover_frames:
                    self.state = self.ACTIVE
                    self.reason = ''
                    self._good_since_pause = 0
            else:
                # 不符合的幀不只是「不算數」，還要把連續計數歸零
                self._good_since_pause = 0
                self.reason = ('recovery frame rejected: age=%.1fs limit=%.1fs'
                               % (now - stamp, self.timeout))
            return (self.state, self.reason)

        return self.tick(now)

    def tick(self, now, _keep_reason=False):
        """沒有新地圖時也要呼叫（時間本身會讓地圖過期）。回傳 (狀態, 原因)。"""
        if not _keep_reason and self._check_clock(now):
            return (self.state, self.reason)

        if self.last_stamp is None:
            return (self.state, self.reason)

        age = now - self.last_stamp
        if age > self.timeout:
            self._pause('map stale: %.1fs > %.1fs' % (age, self.timeout), now)
        elif self.state == self.ACTIVE and not _keep_reason:
            self.reason = ''
        return (self.state, self.reason)

    @property
    def active(self):
        return self.state == self.ACTIVE


# ---------------------------------------------------------------------------
# 4. 目標生命週期
# ---------------------------------------------------------------------------

class GoalLifecycle:
    """同一時刻至多一個有效目標。違反這條＝兩個目標同時控制同一台車。

    狀態流：
        IDLE ──send──> PENDING ──accepted──> ACTIVE ──result──> IDLE
                          └─rejected────> IDLE（呼叫端應把該點加黑名單）
        ACTIVE/PENDING ──cancel──> CANCELLING ──**同 generation 的 result 終態**──> IDLE
        CANCELLING 超過 cancel_timeout 沒等到終態 ──> FATAL
        取消被拒 / 取消回應本身出錯 ──> FATAL

    ⚠ **「取消被受理」不等於「舊目標已結束」**（Codex F1，2026-08-11）：
      ROS action 的 cancel 回應只表示 server 收下了取消請求，goal 可能還在 CANCELING
      狀態跑一段時間。舊版收到 cancel 回應就轉 IDLE，於是下一輪 tick 可能送出第二個
      goal——**兩個目標同時驅動同一台車**，正是本狀態機要防的事。
      現在一律等**同 generation 的 result 終態**才回 IDLE。

    FATAL 的意思是「explorer 必須立刻結束行程」，由 launch 的
    OnProcessExit→Shutdown 接手把整串關掉，serial 一斷韌體 watchdog 就停車。
    這是唯一能保證車會停的路徑：光是「不再送新目標」並不能讓已經在跑的目標停下來。

    每個目標帶一個遞增 generation；舊 generation 的回報（late result）一律丟棄，
    否則一個延遲送達的舊結果會把已經取消／已完成的狀態機打回可動狀態。
    """

    IDLE = 'IDLE'
    PENDING = 'PENDING'
    ACTIVE = 'ACTIVE'
    CANCELLING = 'CANCELLING'
    FATAL = 'FATAL'
    DONE = 'DONE'

    def __init__(self, cancel_timeout=DEFAULT_CANCEL_TIMEOUT_S):
        self.cancel_timeout = cancel_timeout
        self.state = self.IDLE
        self.generation = 0
        self.cancel_started = None
        self.fatal_reason = ''

    @property
    def has_active_goal(self):
        """「車可能正在被這個目標驅動」的所有狀態。"""
        return self.state in (self.PENDING, self.ACTIVE, self.CANCELLING)

    def may_send_goal(self):
        return self.state == self.IDLE

    def on_send(self):
        if not self.may_send_goal():
            raise RuntimeError('在 %s 狀態送新目標＝可能同時有兩個目標' % self.state)
        self.generation += 1
        self.state = self.PENDING
        return self.generation

    def on_accepted(self, gen):
        if gen != self.generation or self.state != self.PENDING:
            return False                      # late／不相干的回報，丟棄
        self.state = self.ACTIVE
        return True

    def on_rejected(self, gen):
        if gen != self.generation or self.state != self.PENDING:
            return False
        self.state = self.IDLE
        return True

    def on_result(self, gen):
        """目標跑完（成功／失敗／被取消都算）。回傳是否採納。"""
        if gen != self.generation:
            return False                      # late result：舊目標的回報，丟棄
        if self.state in (self.FATAL, self.DONE):
            return False                      # 收工後不得復活
        self.state = self.IDLE
        self.cancel_started = None
        return True

    def request_cancel(self, now):
        """要求取消現有目標。取消確認到達前不得送新目標。"""
        if self.state in (self.PENDING, self.ACTIVE):
            self.state = self.CANCELLING
            self.cancel_started = now
            return True
        return False

    def on_cancel_accepted(self, gen):
        """server 受理了取消請求。**維持 CANCELLING**，直到 result 終態才算真的結束。

        回傳是否為當前世代的有效回應（供呼叫端記錄用）。
        """
        return gen == self.generation and self.state == self.CANCELLING

    def on_cancel_rejected(self, gen, reason='cancel rejected by server'):
        """取消被拒 → 致命。我們已經無法停止那個仍在跑的目標。"""
        if gen != self.generation or self.state != self.CANCELLING:
            return False
        self._fatal(reason)
        return True

    def on_server_lost(self, now):
        """action server 不見了。回傳 True＝致命，呼叫端必須結束行程。

        分兩種：手上沒有目標時只要等它回來；**手上有目標卻失聯＝致命**——
        那個目標可能還在 server 端跑著、車還在動，而我們已經無法取消它。
        重連後另派新目標會變成兩個目標同時有效，比直接停車危險得多。
        """
        if self.has_active_goal:
            self._fatal('action server lost while a goal was in flight')
            return True
        return False

    def tick(self, now):
        """每輪呼叫。回傳 fatal 原因字串或 None。"""
        if self.state == self.FATAL:
            return self.fatal_reason
        if self.state == self.CANCELLING and self.cancel_started is not None:
            if now - self.cancel_started > self.cancel_timeout:
                # 逾時的判準是「沒拿到終態」，不是「沒拿到取消回應」（Codex F1）
                self._fatal('goal did not reach a terminal state within %.1fs of cancel'
                            % self.cancel_timeout)
                return self.fatal_reason
        return None

    def finish(self):
        """探索完成。之後任何舊目標回報都不得讓車再動。"""
        self.state = self.DONE

    def _fatal(self, reason):
        self.state = self.FATAL
        self.fatal_reason = reason
