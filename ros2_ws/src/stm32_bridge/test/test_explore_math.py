"""explore_math 的邊界情境測試（EX2，2026-08-11；零 ROS，Windows 直接 pytest）。

地圖用字串畫，由上到下對應 y 由大到小（視覺方向＝俯視圖）：
    '#' 障礙   '.' 已知空地   '?' 未知
"""
import math

import pytest

from stm32_bridge.explore_math import (
    Blacklist, Frontier, GoalLifecycle, Grid, MapHealth,
    choose_goal, cluster_cells, extract_frontiers, find_frontier_cells,
    goal_for_frontier, should_preempt,
    DEFAULT_MIN_FRONTIER_CELLS, DEFAULT_RETREAT_M, DEFAULT_BLACKLIST_RADIUS_M,
    DEFAULT_CANCEL_TIMEOUT_S, DEFAULT_MAP_STALE_TIMEOUT_S,
)

CH = {'#': 100, '.': 0, '?': -1}


def grid_from(rows, resolution=0.05, origin=(0.0, 0.0)):
    """字串列表 → Grid。第一個字串是最上面一列（y 最大）。"""
    h = len(rows)
    w = len(rows[0])
    assert all(len(r) == w for r in rows), '每列長度必須一致'
    data = []
    for j in range(h):                       # j=0 是 y 最小 → 取字串的最後一列
        data.extend(CH[c] for c in rows[h - 1 - j])
    return Grid(data, w, h, resolution, origin[0], origin[1])


# ---------------------------------------------------------------- 政策常數鎖

def test_policy_constants_are_locked():
    """這些值有實體意義（車身尺寸／可達性），改動必須連同理由一起改測試。"""
    assert DEFAULT_MIN_FRONTIER_CELLS == 6          # 0.3 m > 車身寬 0.191 m
    assert DEFAULT_RETREAT_M == 0.5
    assert DEFAULT_BLACKLIST_RADIUS_M == 0.5        # ≥ 車身長 0.337 m
    assert DEFAULT_CANCEL_TIMEOUT_S == 3.0
    assert DEFAULT_MAP_STALE_TIMEOUT_S == 10.0


# ---------------------------------------------------------------- 1. 邊界抽取

def test_single_frontier_found_at_free_unknown_border():
    g = grid_from([
        '##########',
        '#........#',
        '#........#',
        '#........#',
        '??????????',
    ])
    cells = find_frontier_cells(g)
    assert cells, '空地與未知區之間必須抽得出邊界'
    # 邊界應該全部落在最底下那排空地（j=1，因為 j=0 整排是未知）
    assert all(j == 1 for (_, j) in cells), '邊界格應緊貼未知區'
    assert len(cells) == 8


def test_diagonal_only_contact_is_not_a_frontier():
    """只有斜角碰到未知區不算邊界——格子世界的斜角是穿過牆角，車過不去。"""
    g = grid_from([
        '#####',
        '#...#',
        '#..?#',      # 中心 (2,1) 只在斜角方向碰到 '?'（正右是 '?' 所以要小心選點）
        '#####',
    ])
    cells = set(find_frontier_cells(g))
    # (1,1) 與 '?' 只有斜角相鄰 → 不該入選；(2,1) 正右方是 '?' → 該入選
    assert (2, 1) in cells
    assert (1, 1) not in cells


def test_occupied_cell_is_never_a_frontier():
    g = grid_from([
        '###',
        '#.?',
        '???',
    ])
    for (i, j) in find_frontier_cells(g):
        assert g.is_free(i, j), '障礙格不可能是邊界（車不能站在牆裡）'


def test_fully_known_map_yields_no_frontier():
    g = grid_from([
        '#####',
        '#...#',
        '#...#',
        '#####',
    ])
    assert find_frontier_cells(g) == []
    assert extract_frontiers(g) == []


def test_two_separate_clusters_are_not_merged():
    g = grid_from([
        '??????????',
        '..........',      # 上方一條邊界
        '..........',
        '####..####',      # 中間被牆分開
        '..........',
        '??????????',      # 下方一條邊界
    ])
    clusters = cluster_cells(find_frontier_cells(g))
    assert len(clusters) == 2, '上下兩條邊界之間隔著牆，不能併成一叢'


def test_small_clusters_are_filtered_out():
    g = grid_from([
        '##########',
        '#........#',
        '#........#',
        '#..?.....#',      # 單獨一格未知 → 只會生出 4 格邊界（< 門檻 6）
        '#........#',
        '##########',
    ])
    assert find_frontier_cells(g), '確實有邊界格'
    assert extract_frontiers(g) == [], '太小的叢集要被丟掉（掃描縫隙不是值得跑一趟的地方）'
    assert len(extract_frontiers(g, min_cells=1)) == 1, '放寬門檻就抓得到'


# ---------------------------------------------------------------- 2. 挑目標

def _wide_map(unknown_top=True, unknown_bottom=True):
    """左右封牆、上下各一個通往未知區的開口；高 2.0 m，退避 0.5 m 有足夠空間。

    ⚠ 測試地圖一定要用牆把已知區包起來：Grid.at() 越界回未知，
    一整片沒有牆的空地會讓地圖外圈整圈變成邊界（2026-08-11 首版測試就是這樣寫錯的）。
    """
    top = '?' * 31 if unknown_top else '#' * 31
    bot = '?' * 31 if unknown_bottom else '#' * 31
    return grid_from([top] + ['#' + '.' * 29 + '#'] * 18 + [bot], resolution=0.1)


def test_map_edge_counts_as_unexplored():
    """地圖陣列的邊界外視為未知＝貼著地圖邊緣的空地就是邊界，車該往那邊繼續探索。

    這是刻意的語意（slam_toolbox 的地圖會隨探索長大），不是疏漏。
    """
    g = grid_from(['.' * 8] * 8, resolution=0.1)      # 一整片空地，四周沒有牆
    cells = find_frontier_cells(g)
    assert len(cells) == 28, '8x8 的外圈共 28 格，全部都該是邊界'
    inner = [(i, j) for (i, j) in cells if 0 < i < 7 and 0 < j < 7]
    assert inner == [], '內部的格子四鄰都是已知空地，不該是邊界'


def test_goal_retreats_into_known_space_and_faces_the_unknown():
    """目標點要退到已知側，而且車頭朝向未知區。"""
    g = grid_from(
        ['?' * 21] + ['#' + '.' * 19 + '#'] * 14 + ['#' * 21],
        resolution=0.1)
    fs = extract_frontiers(g)
    assert len(fs) == 1
    goal = goal_for_frontier(g, fs[0])
    assert goal is not None
    gx, gy, yaw = goal
    cx, cy = fs[0].centroid
    # 退往已知側＝y 變小（未知區在上方）
    assert gy < cy, '目標必須退到已知空地那一側'
    assert math.isclose(math.hypot(gx - cx, gy - cy), DEFAULT_RETREAT_M, abs_tol=0.06)
    # 朝向未知區＝+y 方向（+90 度）
    assert math.isclose(math.degrees(yaw), 90.0, abs_tol=15.0), \
        '車頭要朝未知區，掃描面才吃得到新區域'


def test_goal_is_none_when_retreat_lands_in_obstacle():
    """退避方向被牆堵死時，這個叢集只能放棄，不能硬給一個撞牆的目標。"""
    g = grid_from([
        '?????????',
        '.........',
        '#########',      # 空地只有薄薄一層，退不進去
        '#########',
        '#########',
    ], resolution=0.1)
    fs = extract_frontiers(g)
    assert fs, '有邊界'
    assert goal_for_frontier(g, fs[0]) is None


def test_choose_goal_prefers_the_nearest():
    g = _wide_map()
    fs = extract_frontiers(g)
    assert len(fs) == 2, '上下各一條邊界'
    # 車貼近下方邊界
    goal, f = choose_goal(g, fs, robot_xy=(1.5, 0.25))
    assert goal is not None
    # 地圖高 2.0 m，下方目標退避後約 y=0.65、上方約 y=1.35 → 以中線 1.0 分辨選了哪一個
    assert goal[1] < 1.0, '應選下方那條（離車近），而不是上方那條'
    assert f.centroid[1] < 1.0


def test_blacklisted_goal_is_skipped_and_next_one_chosen():
    g = _wide_map()
    fs = extract_frontiers(g)
    near, _ = choose_goal(g, fs, robot_xy=(1.5, 0.25))
    bl = Blacklist()
    bl.add(near[0], near[1])
    other, _ = choose_goal(g, fs, robot_xy=(1.5, 0.25), blacklist=bl)
    assert other is not None, '一個目標進黑名單後要能改挑另一個'
    assert math.hypot(other[0] - near[0], other[1] - near[1]) > DEFAULT_BLACKLIST_RADIUS_M


def test_all_goals_blacklisted_means_no_goal():
    g = grid_from(['?' * 21] + ['#' + '.' * 19 + '#'] * 14 + ['#' * 21], resolution=0.1)
    fs = extract_frontiers(g)
    assert len(fs) == 1, '只有上方一個開口'
    goal, _ = choose_goal(g, fs, robot_xy=(1.0, 0.3))
    bl = Blacklist()
    bl.add(goal[0], goal[1])
    assert choose_goal(g, fs, robot_xy=(1.0, 0.3), blacklist=bl) == (None, None)


def test_blacklist_radius_semantics():
    bl = Blacklist(radius=0.5)
    bl.add(1.0, 1.0)
    assert bl.contains(1.0, 1.4)
    assert not bl.contains(1.0, 1.6)


# ---------------------------------------------------------------- 3. 重評估

def test_preempt_when_target_frontier_disappears():
    g1 = _wide_map()
    fs1 = extract_frontiers(g1)
    goal, f = choose_goal(g1, fs1, robot_xy=(1.5, 0.25))

    # 下方那塊掃完了（不再是未知）→ 目標所屬叢集消失
    g2 = _wide_map(unknown_bottom=False)
    fs2 = extract_frontiers(g2)
    do, why = should_preempt(goal, f.centroid, g2, fs2, robot_xy=(1.5, 0.25))
    assert do and 'gone' in why


def test_no_preempt_when_target_still_there_and_nothing_much_closer():
    g = _wide_map()
    fs = extract_frontiers(g)
    goal, f = choose_goal(g, fs, robot_xy=(1.5, 0.25))
    do, why = should_preempt(goal, f.centroid, g, fs, robot_xy=(1.5, 0.25))
    assert not do, '目標還在、也沒有明顯更近的，不該換來換去（換目標要先取消，有成本）'


def test_preempt_when_a_much_closer_frontier_appears():
    g = _wide_map()
    fs = extract_frontiers(g)
    # 車靠近下方，但假裝現在的目標是上方那個（遠的）
    far = [f for f in fs if f.centroid[1] > 0.5][0]
    far_goal = goal_for_frontier(g, far)
    do, why = should_preempt(far_goal, far.centroid, g, fs, robot_xy=(1.5, 0.15))
    assert do and 'closer' in why


# ---------------------------------------------------------------- 4. 地圖過期

def test_map_health_normal_flow():
    m = MapHealth()
    assert m.on_map(stamp=100.0, now=100.1)[0] == MapHealth.ACTIVE
    assert m.tick(now=105.0)[0] == MapHealth.ACTIVE
    assert m.active


def test_map_health_times_out():
    m = MapHealth(timeout=10.0)
    m.on_map(stamp=100.0, now=100.0)
    assert m.tick(now=109.9)[0] == MapHealth.ACTIVE
    st, why = m.tick(now=110.5)
    assert st == MapHealth.PAUSED and 'stale' in why


def test_map_health_rejects_future_stamp():
    """時戳超前現在太多＝時鐘沒同步，會讓地圖永遠看起來很新而遮蔽真正的過期。"""
    m = MapHealth(timeout=10.0)
    m.on_map(stamp=100.0, now=100.0)
    st, why = m.on_map(stamp=200.0, now=105.0)      # 超前 95 秒
    assert 'future' in why
    assert m.last_stamp == 100.0, '非法幀不得被採納為最新時戳'
    st, _ = m.tick(now=111.0)
    assert st == MapHealth.PAUSED, '非法幀不算數，時間照走 → 應該過期'


def test_map_health_rejects_zero_and_nan_stamp():
    m = MapHealth(timeout=10.0)
    m.on_map(stamp=100.0, now=100.0)
    _, why = m.on_map(stamp=0.0, now=101.0)
    assert 'zero' in why
    _, why = m.on_map(stamp=float('nan'), now=102.0)
    assert 'finite' in why
    _, why = m.on_map(stamp=float('inf'), now=103.0)
    assert 'finite' in why
    assert m.last_stamp == 100.0


def test_duplicate_stamp_does_not_reset_the_clock():
    """一個卡住的節點反覆重發同一幀，不能因此永遠騙過過期偵測。"""
    m = MapHealth(timeout=10.0)
    m.on_map(stamp=100.0, now=100.0)
    for t in (102.0, 104.0, 106.0, 108.0):
        m.on_map(stamp=100.0, now=t)            # 同一個時戳一直重發
        assert m.active
    st, why = m.on_map(stamp=100.0, now=111.0)
    assert st == MapHealth.PAUSED and 'stale' in why


def test_older_stamp_does_not_reset_the_clock():
    m = MapHealth(timeout=10.0)
    m.on_map(stamp=100.0, now=100.0)
    m.on_map(stamp=95.0, now=105.0)             # 更舊的一幀
    assert m.last_stamp == 100.0
    assert m.tick(now=111.0)[0] == MapHealth.PAUSED


def test_clock_going_backwards_pauses_immediately():
    """/clock 被重置時，所有年齡計算的基準都不可信了，必須立刻停手。"""
    m = MapHealth(timeout=10.0)
    m.on_map(stamp=100.0, now=100.0)
    st, why = m.tick(now=50.0)
    assert st == MapHealth.PAUSED and 'backwards' in why


def test_recovery_needs_two_fresh_frames():
    m = MapHealth(timeout=10.0, recover_frames=2)
    m.on_map(stamp=100.0, now=100.0)
    assert m.tick(now=111.0)[0] == MapHealth.PAUSED
    assert m.on_map(stamp=111.5, now=111.6)[0] == MapHealth.PAUSED, '一幀不夠'
    assert m.on_map(stamp=112.0, now=112.1)[0] == MapHealth.ACTIVE, '連續兩幀才恢復'


def test_recovery_requires_frames_newer_than_pause_point():
    """PAUSED 之後，比進入 PAUSED 當時還舊的時戳不能拿來當恢復證據。"""
    m = MapHealth(timeout=10.0, recover_frames=2)
    m.on_map(stamp=100.0, now=100.0)
    m.tick(now=111.0)
    m.on_map(stamp=99.0, now=111.5)
    m.on_map(stamp=99.5, now=112.0)
    assert m.state == MapHealth.PAUSED, '舊幀不算恢復證據'


# ---------------------------------------------------------------- 5. 目標生命週期

def test_cannot_send_two_goals_at_once():
    gl = GoalLifecycle()
    gl.on_send()
    assert not gl.may_send_goal()
    with pytest.raises(RuntimeError):
        gl.on_send()


def test_normal_goal_cycle():
    gl = GoalLifecycle()
    gen = gl.on_send()
    assert gl.on_accepted(gen)
    assert gl.has_active_goal
    assert gl.on_result(gen)
    assert gl.state == GoalLifecycle.IDLE
    assert gl.may_send_goal()


def test_rejected_goal_returns_to_idle():
    gl = GoalLifecycle()
    gen = gl.on_send()
    assert gl.on_rejected(gen)
    assert gl.may_send_goal(), '被拒絕之後要能改挑別的目標（呼叫端負責加黑名單）'


def test_late_result_from_old_goal_is_discarded():
    """遲到的舊結果不能把狀態機打回可動狀態。"""
    gl = GoalLifecycle()
    g1 = gl.on_send()
    gl.on_accepted(g1)
    gl.request_cancel(now=0.0)
    gl.on_cancel_accepted(g1)
    gl.on_result(g1)                 # 終態到達，第一個目標真的結束了
    g2 = gl.on_send()
    gl.on_accepted(g2)
    assert not gl.on_result(g1), '第一個目標的遲到回報必須丟棄'
    assert gl.state == GoalLifecycle.ACTIVE, '第二個目標仍在執行中'
    assert gl.on_result(g2)


def test_cancel_blocks_new_goals_until_terminal_state():
    """Codex F1：「取消被受理」不等於「舊目標已結束」，必須等 result 終態。"""
    gl = GoalLifecycle()
    gen = gl.on_send()
    gl.on_accepted(gen)
    gl.request_cancel(now=0.0)
    assert not gl.may_send_goal(), '取消期間不得送新目標'
    assert gl.on_cancel_accepted(gen), 'server 受理取消'
    assert not gl.may_send_goal(),         '**受理取消不等於目標已終止**——此時送新目標會變成兩個目標同時驅動同一台車'
    gl.on_result(gen)
    assert gl.may_send_goal(), '拿到終態才解除封鎖'


def test_cancel_rejected_is_fatal():
    """取消被拒＝我們已經無法停止那個仍在跑的目標 → 必須自殺讓 supervisor 停車。"""
    gl = GoalLifecycle()
    gen = gl.on_send()
    gl.on_accepted(gen)
    gl.request_cancel(now=0.0)
    assert gl.on_cancel_rejected(gen) is True
    assert gl.state == GoalLifecycle.FATAL
    assert not gl.may_send_goal()


def test_cancel_accepted_but_never_terminates_is_fatal():
    """取消被受理、但目標遲遲不結束——逾時判準是「沒拿到終態」，不是「沒拿到回應」。"""
    gl = GoalLifecycle(cancel_timeout=3.0)
    gen = gl.on_send()
    gl.on_accepted(gen)
    gl.request_cancel(now=10.0)
    gl.on_cancel_accepted(gen)           # 受理了
    assert gl.tick(now=12.9) is None
    reason = gl.tick(now=13.1)
    assert reason is not None and 'terminal state' in reason
    assert gl.state == GoalLifecycle.FATAL


def test_cancel_never_acknowledged_is_fatal():
    """卡上最重要的一條：取消後拿不到終態時，正確反應是自殺讓車停，不是繼續等。"""
    gl = GoalLifecycle(cancel_timeout=3.0)
    gen = gl.on_send()
    gl.on_accepted(gen)
    gl.request_cancel(now=10.0)
    assert gl.tick(now=12.9) is None
    reason = gl.tick(now=13.1)
    assert reason is not None and 'terminal state' in reason
    assert gl.state == GoalLifecycle.FATAL


def test_server_lost_with_goal_in_flight_is_fatal():
    gl = GoalLifecycle()
    gen = gl.on_send()
    gl.on_accepted(gen)
    assert gl.on_server_lost(now=5.0) is True
    assert gl.state == GoalLifecycle.FATAL


def test_server_lost_while_idle_is_not_fatal():
    gl = GoalLifecycle()
    assert gl.on_server_lost(now=5.0) is False
    assert gl.state == GoalLifecycle.IDLE, '手上沒目標時失聯，等它回來就好'


def test_no_revival_after_exploration_done():
    gl = GoalLifecycle()
    gen = gl.on_send()
    gl.on_accepted(gen)
    gl.finish()
    assert not gl.on_result(gen), '收工後舊目標的回報不得讓狀態機回到可動狀態'
    assert gl.state == GoalLifecycle.DONE
    assert not gl.may_send_goal()


def test_fatal_is_sticky():
    gl = GoalLifecycle(cancel_timeout=1.0)
    gen = gl.on_send()
    gl.on_accepted(gen)
    gl.request_cancel(now=0.0)
    gl.tick(now=2.0)
    assert gl.state == GoalLifecycle.FATAL
    assert gl.tick(now=2.1) is not None, 'FATAL 之後每輪都要繼續回報，不能自己好起來'
    assert not gl.may_send_goal()


# ---------------------------------------------------------------- 6. 無進展的目標

def test_goal_with_no_new_cells_counts_as_no_progress():
    """抵達了卻沒掃到新東西＝這個目標沒用，必須進黑名單，否則無限迴圈。"""
    from stm32_bridge.explore_math import goal_made_progress
    assert goal_made_progress(1000, 1000) is False
    assert goal_made_progress(1000, 999) is False


def test_goal_with_new_cells_counts_as_progress():
    from stm32_bridge.explore_math import goal_made_progress
    assert goal_made_progress(1000, 1001) is True
    assert goal_made_progress(1000, 1500) is True


def test_frontier_goal_accepts_unknown_nearby_but_not_obstacles():
    """挑目標時：周圍有未知區沒關係（那正是要去探索的），有障礙才不行。"""
    g = grid_from(['?' * 21] + ['#' + '.' * 19 + '#'] * 14 + ['#' * 21], resolution=0.1)
    fs = extract_frontiers(g)
    goal = goal_for_frontier(g, fs[0])
    assert goal is not None
    gx, gy, _ = goal
    assert g.no_obstacle_around(gx, gy, 0.25), '目標周圍不得有障礙'
    # 嚴格版（未知也不准）在邊界附近本來就會失敗——這正是它不該拿來挑目標的原因
    assert not g.clear_around(gx, gy, 0.6), \
        '若連遠處的未知都要求已知，探索會走一步就停（2026-08-11 實測 38%）'


def test_retreat_fallback_finds_goal_in_tight_corner():
    """退避 0.5 m 退不進去的狹窄處，要能退少一點再試（否則角落起點一個目標都生不出來）。"""
    from stm32_bridge.explore_math import MIN_RETREAT_M
    # 空地 0.8 m 深：退 0.5 m 的落點周圍 0.25 m 會碰到底牆（不合格），
    # 退 0.35 m 才落在中間、周圍乾淨。
    # ⚠ 別把空地畫得比 2×clearance(0.5 m) 還淺——那樣是物理上不可能合格，
    #   測不到「備援有沒有生效」，只會測到「空間太小」（首版就是這樣寫錯的）。
    g = grid_from(
        ['?' * 15] + ['#' + '.' * 13 + '#'] * 8 + ['#' * 15],
        resolution=0.1)
    fs = extract_frontiers(g)
    assert fs, '有邊界'
    goal = goal_for_frontier(g, fs[0])
    assert goal is not None, '退避備援要能在狹窄處找到目標'
    cx, cy = fs[0].centroid
    d = math.hypot(goal[0] - cx, goal[1] - cy)
    assert d >= MIN_RETREAT_M - 0.06, \
        '退避不得低於下限 %.2f m（車頭前伸 0.28 m，再少就伸進未探索區）' % MIN_RETREAT_M


def test_min_retreat_is_locked_above_front_overhang():
    from stm32_bridge.explore_math import MIN_RETREAT_M
    assert MIN_RETREAT_M >= 0.28, \
        '下限必須 >= 車頭前伸 0.28 m；調低等於允許車頭停在未探索區裡'


# --- Codex F2（2026-08-11）：PAUSED 不可被「遞增但仍然過期」的時戳解除 ---

def test_paused_not_recovered_by_advancing_but_still_stale_frames():
    """地圖舊了 98 秒，只是時戳有在前進——絕不能因此恢復派送目標。"""
    m = MapHealth(timeout=10.0, recover_frames=2)
    m.on_map(stamp=100.0, now=100.0)
    assert m.tick(now=200.0)[0] == MapHealth.PAUSED
    for st in (101.0, 102.0, 103.0):
        state, why = m.on_map(stamp=st, now=200.0)
        assert state == MapHealth.PAUSED, \
            '時戳 %.0f 相對 now=200 已舊 %.0f 秒，不該解除暫停' % (st, 200.0 - st)
    assert 'rejected' in why


def test_paused_recovers_only_with_frames_newer_than_pause_moment():
    """恢復幀必須晚於「進入 PAUSED 的 ROS 時刻」，不是晚於「最後一張地圖的時戳」。"""
    m = MapHealth(timeout=10.0, recover_frames=2)
    m.on_map(stamp=100.0, now=100.0)
    m.tick(now=115.0)                     # 進入 PAUSED，此刻 now=115
    # 時戳 110 比 last_stamp(100) 大、相對 now=116 也才舊 6 秒（未過期），
    # 但它早於進入 PAUSED 的時刻 115 → 仍不得作為恢復證據
    m.on_map(stamp=110.0, now=116.0)
    m.on_map(stamp=111.0, now=116.5)
    assert m.state == MapHealth.PAUSED
    # 晚於 115 且夠新的兩幀才恢復
    m.on_map(stamp=117.0, now=117.1)
    assert m.on_map(stamp=118.0, now=118.1)[0] == MapHealth.ACTIVE


def test_rejected_recovery_frame_resets_the_streak():
    """一好一壞交替不該累積成「連續兩幀」。"""
    m = MapHealth(timeout=10.0, recover_frames=2)
    m.on_map(stamp=100.0, now=100.0)
    m.tick(now=120.0)
    m.on_map(stamp=121.0, now=121.0)      # 好幀（1/2）
    m.on_map(stamp=122.0, now=200.0)      # 壞幀（已舊 78 秒）→ 計數歸零
    assert m.state == MapHealth.PAUSED
    m.on_map(stamp=201.0, now=201.0)      # 好幀（1/2，不是 2/2）
    assert m.state == MapHealth.PAUSED, '壞幀之後要重新累積，不能沿用先前的計數'


# --- 目標結束後要不要列黑名單（2026-09-24 run5：主動換目標被當失敗）---------

def test_judge_arrived_with_gain_not_listed():
    from stm32_bridge.explore_math import judge_goal_result, ARRIVED
    assert judge_goal_result(4, False, 1000, 1200) == (False, ARRIVED)


def test_judge_arrived_without_gain_listed():
    from stm32_bridge.explore_math import judge_goal_result, ARRIVED_NO_GAIN
    assert judge_goal_result(4, False, 1000, 1000) == (True, ARRIVED_NO_GAIN)


def test_judge_our_preempt_with_gain_not_listed():
    """run5 的 4 次「target frontier gone」：邊界被掃掉了才換，不是目標的錯。"""
    from stm32_bridge.explore_math import judge_goal_result, PREEMPTED
    assert judge_goal_result(5, True, 1000, 1500) == (False, PREEMPTED)


def test_judge_our_preempt_without_gain_still_listed():
    """換目標途中地圖沒長大仍然列——防止在兩個目標之間來回換。"""
    from stm32_bridge.explore_math import judge_goal_result, PREEMPTED_NO_GAIN
    assert judge_goal_result(5, True, 1000, 1000) == (True, PREEMPTED_NO_GAIN)


def test_judge_cancel_not_requested_by_preempt_is_failure():
    """不是我們換目標而取消的（例如地圖過期、別人取消）一律當失敗。"""
    from stm32_bridge.explore_math import judge_goal_result, FAILED
    assert judge_goal_result(5, False, 1000, 1500) == (True, FAILED)


def test_judge_aborted_is_failure_even_if_map_grew():
    from stm32_bridge.explore_math import judge_goal_result, FAILED
    assert judge_goal_result(6, True, 1000, 1500) == (True, FAILED)
    assert judge_goal_result(None, True, 1000, 1500) == (True, FAILED)


def test_judge_unknown_before_counts_as_gain():
    """送出時還沒有地圖（known_before=None）＝不以「沒收穫」為由封掉，與舊行為一致。"""
    from stm32_bridge.explore_math import judge_goal_result
    assert judge_goal_result(4, False, None, 0)[0] is False
    assert judge_goal_result(5, True, None, 0)[0] is False
