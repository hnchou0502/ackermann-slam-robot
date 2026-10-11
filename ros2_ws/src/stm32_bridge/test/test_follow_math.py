"""follow_math 的單元測試——零 ROS 依賴，任何機器 `pytest test/test_follow_math.py` 即可跑。

重點測「幾何會不會算錯」，特別是：
  - 阿克曼可行域（轉不過去的指令必須被夾住，不是畫出不可能的軌跡）
  - 8 字參數與實際積分出來的軌跡一致（車頭確實轉滿一圈再轉回來）
  - 麵包屑的沿路徑弧長（轉彎時直線距離會低估跟車距離，這是切內線的根源）
"""
import math

import pytest

from stm32_bridge.follow_math import (
    BreadcrumbTrail,
    LinkMonitor,
    V2V_SEQ_COV_INDEX,
    adaptive_lookahead,
    align_leader_point,
    apply_ackermann,
    cross_track_series,
    figure8_omega,
    figure8_params,
    min_turn_radius,
    normalize_angle,
    point_segment_distance,
    pure_pursuit_curvature,
    rms,
    score_following,
    world_to_base,
    yaw_from_quaternion,
)

L = 0.235                    # 本車軸距 (m)
MAX_STEER = math.radians(30.0)


# --- normalize_angle ---------------------------------------------------------

def test_normalize_angle_wraps_to_pm_pi():
    assert normalize_angle(0.0) == pytest.approx(0.0)
    assert normalize_angle(2 * math.pi + 0.5) == pytest.approx(0.5, abs=1e-9)
    # ±pi 落在區間邊界上，兩者等價；atan2 對 -3pi 回 -pi、對 +3pi 回 +pi。
    # 只驗「大小是 pi」，不綁符號。
    assert abs(normalize_angle(3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert abs(normalize_angle(-3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)


# --- 阿克曼幾何 ---------------------------------------------------------------

def test_min_turn_radius_matches_hand_calc():
    # L / tan(30deg) = 0.235 / 0.57735 = 0.40704 m
    assert min_turn_radius(L, MAX_STEER) == pytest.approx(0.40704, abs=1e-5)


def test_ackermann_straight_line_unchanged():
    v, w = apply_ackermann(0.3, 0.0, L, MAX_STEER)
    assert (v, w) == pytest.approx((0.3, 0.0))


def test_ackermann_cannot_spin_in_place():
    """|v| 太小時角速度必須歸零——前輪轉向的車不能原地旋轉。"""
    v, w = apply_ackermann(0.005, 2.0, L, MAX_STEER, min_turn_speed=0.01)
    assert w == 0.0
    assert v == pytest.approx(0.005)


def test_ackermann_clamps_over_curvature():
    """要求半徑 0.06m（遠小於 R_min 0.407m）-> 角速度被夾到 v/R_min。"""
    v_in, w_in = 0.3, 5.0
    v, w = apply_ackermann(v_in, w_in, L, MAX_STEER)
    w_max = v_in / min_turn_radius(L, MAX_STEER)
    assert v == pytest.approx(v_in)
    assert w == pytest.approx(w_max, rel=1e-6)
    assert w < w_in


def test_ackermann_symmetric_for_left_and_right():
    _, w_left = apply_ackermann(0.3, 5.0, L, MAX_STEER)
    _, w_right = apply_ackermann(0.3, -5.0, L, MAX_STEER)
    assert w_left == pytest.approx(-w_right)


def test_ackermann_within_feasible_region_is_untouched():
    """半徑 0.9m 的指令在可行域內，不該被動到。"""
    v_in, w_in = 0.35, 0.35 / 0.9
    v, w = apply_ackermann(v_in, w_in, L, MAX_STEER)
    assert w == pytest.approx(w_in, rel=1e-9)


# --- 8 字軌跡 ----------------------------------------------------------------

def test_figure8_params_hand_calc():
    a, t = figure8_params(0.35, 0.9)
    assert a == pytest.approx(0.35 / 0.9, rel=1e-9)          # 0.38889 rad/s
    assert t == pytest.approx(2 * math.pi ** 2 / a, rel=1e-9)  # 50.76 s


def test_figure8_params_rejects_nonpositive():
    with pytest.raises(ValueError):
        figure8_params(0.35, 0.0)
    with pytest.raises(ValueError):
        figure8_params(0.0, 0.9)


def test_figure8_peak_omega_gives_requested_radius():
    """瞬時最小轉彎半徑 = v / max|w| 必須等於要求的 radius。"""
    v, radius = 0.35, 0.9
    a, t = figure8_params(v, radius)
    w_peak = max(abs(figure8_omega(i * t / 2000.0, a, t)) for i in range(2001))
    assert v / w_peak == pytest.approx(radius, rel=1e-3)


def test_figure8_integrates_to_full_heading_loop_and_back():
    """積分一個週期：車頭轉滿一圈（峰值 2pi）後回到 0——這才是「兩個環＝8 字」。"""
    v, radius = 0.35, 0.9
    a, t = figure8_params(v, radius)
    dt = t / 20000.0
    th = 0.0
    th_peak = 0.0
    for i in range(20000):
        th += figure8_omega(i * dt, a, t) * dt
        th_peak = max(th_peak, th)
    assert th_peak == pytest.approx(2 * math.pi, rel=2e-3)
    assert th == pytest.approx(0.0, abs=2e-3)


def test_figure8_stays_inside_ackermann_feasible_region():
    """整條 8 字的每一刻都不該被阿克曼夾限動到，否則實車畫不出這個形狀。"""
    v, radius = 0.35, 0.9
    a, t = figure8_params(v, radius)
    assert radius > min_turn_radius(L, MAX_STEER)
    for i in range(2001):
        w_cmd = figure8_omega(i * t / 2000.0, a, t)
        _, w_actual = apply_ackermann(v, w_cmd, L, MAX_STEER)
        assert w_actual == pytest.approx(w_cmd, abs=1e-9)


# --- 座標轉換與純追蹤 ---------------------------------------------------------

def test_world_to_base_rotates_into_robot_frame():
    # 車在 (1,1) 朝北（yaw=90deg）；世界點 (1,2) 在它正前方 1m
    bx, by = world_to_base(1.0, 2.0, 1.0, 1.0, math.pi / 2)
    assert bx == pytest.approx(1.0, abs=1e-9)
    assert by == pytest.approx(0.0, abs=1e-9)


def test_world_to_base_point_on_robot_is_origin():
    bx, by = world_to_base(2.0, -3.0, 2.0, -3.0, 0.7)
    assert (bx, by) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_pure_pursuit_straight_ahead_is_zero_curvature():
    assert pure_pursuit_curvature(1.0, 0.0) == pytest.approx(0.0)


def test_pure_pursuit_left_target_turns_left():
    """by>0（目標在左）-> kappa>0 -> 左轉，與 T3 實機驗證的 wz>0 前輪左偏一致。"""
    assert pure_pursuit_curvature(1.0, 1.0) > 0.0
    assert pure_pursuit_curvature(1.0, -1.0) < 0.0


def test_pure_pursuit_curvature_hand_calc():
    # kappa = 2*by/ld^2 ; (bx,by)=(0,1) -> ld^2=1 -> kappa=2
    assert pure_pursuit_curvature(0.0, 1.0) == pytest.approx(2.0)
    # 半徑 = 1/kappa = 0.5
    assert 1.0 / pure_pursuit_curvature(0.0, 1.0) == pytest.approx(0.5)


def test_pure_pursuit_degenerate_target_returns_zero():
    assert pure_pursuit_curvature(0.0, 0.0) == 0.0


# --- 麵包屑軌跡 ---------------------------------------------------------------

def test_breadcrumb_rejects_points_closer_than_min_spacing():
    trail = BreadcrumbTrail(min_spacing=0.05)
    assert trail.append(0.0, 0.0) is True
    assert trail.append(0.01, 0.0) is False    # 太近，不記
    assert trail.append(0.06, 0.0) is True
    assert len(trail) == 2


def test_breadcrumb_accumulates_arc_length():
    trail = BreadcrumbTrail(min_spacing=0.05)
    for i in range(11):
        trail.append(i * 0.1, 0.0)
    assert trail.total_length == pytest.approx(1.0, abs=1e-9)


def test_breadcrumb_path_gap_is_remaining_arc_length():
    trail = BreadcrumbTrail(min_spacing=0.05)
    for i in range(11):
        trail.append(i * 0.1, 0.0)          # 一條 0->1.0m 的直線
    assert trail.path_gap(0.0, 0.0) == pytest.approx(1.0, abs=1e-9)
    assert trail.path_gap(0.7, 0.0) == pytest.approx(0.3, abs=1e-9)
    assert trail.path_gap(1.0, 0.0) == pytest.approx(0.0, abs=1e-9)


def test_breadcrumb_path_gap_counts_distance_to_trail_when_off_trail():
    """demo 開場：follower 在 leader 起點後方 1.0m，軌跡還沒長出來。

    此時剩餘弧長是 0，但真正的跟車距離是 1.0m。只算弧長會謊報「已貼上 leader」。
    """
    trail = BreadcrumbTrail(min_spacing=0.05)
    trail.append(1.0, 0.0)                   # leader 起點（follower 在原點）
    assert trail.total_length == pytest.approx(0.0)
    assert trail.path_gap(0.0, 0.0) == pytest.approx(1.0, abs=1e-9)

    # leader 往前走 0.5m -> 跟車距離變 1.5m
    trail.append(1.5, 0.0)
    assert trail.path_gap(0.0, 0.0) == pytest.approx(1.5, abs=1e-9)


def test_breadcrumb_path_gap_exceeds_straight_line_on_a_curve():
    """轉彎時沿路徑距離 > 直線距離——這正是不能用直線距離當跟車間距的原因。"""
    trail = BreadcrumbTrail(min_spacing=0.01)
    r = 1.0
    for i in range(91):                      # 沿單位圓走 90 度
        a = math.radians(i)
        trail.append(r * math.sin(a), r * (1 - math.cos(a)))
    arc = trail.path_gap(0.0, 0.0)
    straight = math.hypot(trail.pts[-1][0], trail.pts[-1][1])
    assert arc == pytest.approx(math.pi / 2, abs=0.02)   # 四分之一圓弧長
    assert arc > straight + 0.1                          # 弧長 1.571 vs 弦長 1.414


def test_breadcrumb_lookahead_point_advances_along_path():
    trail = BreadcrumbTrail(min_spacing=0.05)
    for i in range(21):
        trail.append(i * 0.1, 0.0)          # 0 -> 2.0m 直線
    tx, ty = trail.lookahead_point(0.0, 0.0, 0.8)
    assert tx == pytest.approx(0.8, abs=0.051)
    assert ty == pytest.approx(0.0, abs=1e-9)


def test_breadcrumb_lookahead_saturates_at_leader_position():
    """lookahead 超出軌跡末端時，目標就是 leader 目前位置（不外插）。"""
    trail = BreadcrumbTrail(min_spacing=0.05)
    for i in range(11):
        trail.append(i * 0.1, 0.0)          # 只有 1.0m
    tx, ty = trail.lookahead_point(0.0, 0.0, 5.0)
    assert (tx, ty) == pytest.approx((1.0, 0.0), abs=1e-9)


def test_breadcrumb_lookahead_on_empty_trail_is_none():
    assert BreadcrumbTrail().lookahead_point(0.0, 0.0, 0.8) is None


def _figure8_poses(dt=0.01, v=0.35, radius=0.9):
    """積分出一整條 8 字軌跡的位姿點（會自我交叉）。"""
    a, period = figure8_params(v, radius)
    x = y = th = 0.0
    poses = []
    for i in range(int(period / dt)):
        w = figure8_omega(i * dt, a, period)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        poses.append((x, y))
    return poses


def test_figure8_trail_self_intersects():
    """前提確認：8 字軌跡真的會自我交叉（不然下面那個迴歸測試沒有意義）。"""
    poses = _figure8_poses()
    n = len(poses)
    # 找一對「弧長差很遠、但空間位置很近」的點 = 交叉
    found = False
    for i in range(0, n, 20):
        for j in range(i + n // 3, n, 20):
            if math.hypot(poses[i][0] - poses[j][0], poses[i][1] - poses[j][1]) < 0.1:
                found = True
                break
        if found:
            break
    assert found, '8 字軌跡應該要自我交叉'


def test_path_gap_stays_stable_across_self_intersection():
    """迴歸測試（2026-07-10 實測 bug）：follower 沿 leader 軌跡落後固定弧長。

    真實跟車距離全程恆定，path_gap 就該全程恆定。修復前，follower 開過 8 字交叉點時
    最近點會跳到「半圈前的另一條分支」，path_gap 從 1.2m 暴衝到 7.4m。
    """
    poses = _figure8_poses()
    step_len = 0.35 * 0.01                 # 每步前進距離 (m)
    lag_steps = int(1.2 / step_len)        # 落後 1.2m 弧長

    trail = BreadcrumbTrail(min_spacing=0.02)
    gaps = []
    for i, p in enumerate(poses):
        trail.append(*p)
        j = i - lag_steps
        if j < 0:
            continue
        gaps.append(trail.path_gap(*poses[j]))

    assert len(gaps) > 1000
    # 量化門檻：所有取樣的跟車距離都該落在 1.2m 附近（±0.1m 容忍麵包屑量化與弧長離散）
    assert max(gaps) < 1.35, f'path_gap 在自交點爆掉了：max={max(gaps):.2f} m'
    assert min(gaps) > 1.05, f'path_gap 掉太低：min={min(gaps):.2f} m'
    # 相鄰取樣不該有跳變
    jumps = [abs(b - a) for a, b in zip(gaps, gaps[1:])]
    assert max(jumps) < 0.1, f'path_gap 有跳變：max jump={max(jumps):.3f} m'


def test_closest_index_window_ignores_far_branch():
    """兩條空間上貼近、但弧長相距很遠的分支：投影點必須留在當前分支。"""
    trail = BreadcrumbTrail(min_spacing=0.05, search_fwd=2.0)
    for i in range(41):                  # 分支 A：沿 y=0 前進 4m
        trail.append(i * 0.1, 0.0)
    for i in range(41):                  # 分支 B：折返，沿 y=0.03 貼著 A 回來
        trail.append(4.0 - i * 0.1, 0.03)

    trail.reset_cursor()
    i = trail.closest_index(1.0, 0.0)    # 車在分支 A 上的 (1.0, 0)
    assert trail.pts[i][2] == pytest.approx(1.0, abs=0.06)   # 弧長 ~1.0m，屬於分支 A
    # 若是全域搜尋，這裡會跳到分支 B 的對應點（弧長 ~7.0m）
    assert trail.pts[i][2] < 2.0


def test_breadcrumb_prune_keeps_trail_bounded_and_preserves_gap():
    trail = BreadcrumbTrail(min_spacing=0.05)
    for i in range(101):
        trail.append(i * 0.1, 0.0)          # 10m 直線
    # 模擬 follower 沿軌跡一路開到 9m（cursor 隨之推進）——這是實際使用模式
    for i in range(19):
        trail.path_gap(i * 0.5, 0.0)

    gap_before = trail.path_gap(9.0, 0.0)
    assert gap_before == pytest.approx(1.0, abs=1e-9)
    n_before = len(trail)
    trail.prune_before(9.0, 0.0, keep_back=1.0)
    assert len(trail) < n_before                          # 真的有刪
    assert trail.path_gap(9.0, 0.0) == pytest.approx(gap_before, abs=1e-9)   # 間距不受影響


def test_closest_index_window_requires_progressive_queries():
    """釘住窗口法的已知限制：投影點一次最多前進 search_fwd 的弧長。

    正常使用（follower 沿軌跡開）不會踩到，因為每個控制週期只前進幾公分。
    但若 follower 節點重啟、或位置突然跳到軌跡遠處，cursor 會落後 -> 投影錯位。
    真的需要跳躍時要先 reset_cursor()，再連續查詢讓 cursor 追上。
    """
    trail = BreadcrumbTrail(min_spacing=0.05, search_fwd=5.0)
    for i in range(101):
        trail.append(i * 0.1, 0.0)          # 10m 直線

    # cursor 停在 0，直接問 9m 處 -> 只搜得到 5m 內，投影落在窗口邊界
    assert trail.path_gap(9.0, 0.0) == pytest.approx(9.0, abs=0.1)   # 4m 直線距離 + 5m 剩餘弧長

    # 沿軌跡逐步查詢，cursor 追上後答案就正確
    trail.reset_cursor()
    for i in range(19):
        trail.path_gap(i * 0.5, 0.0)
    assert trail.path_gap(9.0, 0.0) == pytest.approx(1.0, abs=1e-9)


# --- 驗收評分（score_follow 抽出來的純數學）----------------------------------

def test_rms_is_true_rms_not_mean():
    # 3,4 的 RMS = sqrt((9+16)/2)=3.5355，平均會是 3.5
    assert rms([3.0, 4.0]) == pytest.approx(math.sqrt(12.5))
    assert math.isnan(rms([]))                      # 空序列回 nan，不回 0


def test_point_segment_distance_perpendicular_and_endpoint():
    # 點在線段上方，垂足落在段內 -> 垂直距離
    assert point_segment_distance(0.5, 0.3, 0.0, 0.0, 1.0, 0.0) == pytest.approx(0.3)
    # 垂足落在段外 -> 取端點距離
    assert point_segment_distance(2.0, 0.0, 0.0, 0.0, 1.0, 0.0) == pytest.approx(1.0)


def test_cross_track_series_single_point_returns_nan_not_crash():
    """回歸：leader 折線只有 1 點時舊版 min() 對空序列丟 ValueError 崩潰。"""
    out = cross_track_series([(0.0, 0.0)], [(1.0, 1.0)])
    assert len(out) == 1 and math.isnan(out[0])
    # 兩點才構成折線
    out2 = cross_track_series([(0.5, 0.3)], [(0.0, 0.0), (1.0, 0.0)])
    assert out2[0] == pytest.approx(0.3)


def _good_rows(n=400, dt=0.05, err=0.02, center=1.2):
    """n 筆「完美跟隨」樣本：follower 貼在 leader 直線軌跡上、間距恆定。"""
    rows = []
    for i in range(n):
        t = i * dt
        lx = i * dt          # leader 沿 x 前進
        fx = lx - center     # follower 落後 center 公尺，同一條線上 (fy=0)
        rows.append((t, fx, 0.0, lx, 0.0, center, err, center))
    return rows


def _score(rows, **kw):
    p = dict(rms_limit=0.3, collision_dist=0.337, xte_limit=0.15, settle=10.0,
             leader_length=20.0, min_samples=100, min_leader_length=1.0)
    p.update(kw)
    return score_following(rows, **p)


def test_score_good_run_passes():
    assert _score(_good_rows())['code'] == 0


def test_score_few_samples_never_passes():
    """回歸（假 PASS 主雷）：只有幾筆『完美』樣本，指標算起來全綠，但樣本不足必須擋成 code 2。"""
    res = _score(_good_rows(n=5))
    assert res['code'] == 2                          # 不是 0
    assert '樣本不足' in res['reason']


def test_score_leader_not_moving_never_passes():
    """leader 幾乎沒動（軌跡總長太短）時不予評分，不能因為 follower 也沒動就判 PASS。"""
    res = _score(_good_rows(), leader_length=0.3)
    assert res['code'] == 2
    assert 'leader' in res['reason']


def test_score_collision_fails():
    rows = _good_rows()
    rows[200] = rows[200][:7] + (0.2,)               # 某刻中心距 0.2 < 車長 0.337
    res = _score(rows)
    assert res['code'] == 1 and res['pass_collision'] is False


def test_score_cutting_corner_fails_on_cross_track():
    """間距與碰撞都過、但 follower 偏離 leader 路徑（切內線）-> cross-track 判 FAIL。"""
    rows = _good_rows()
    # settle(10s) 之後把 follower 橫向拉開 0.4m（> xte_limit 0.15）
    for i in range(250, 400):
        t, fx, _, lx, ly, gap, err, cd = rows[i]
        rows[i] = (t, fx, 0.4, lx, ly, gap, err, math.hypot(lx - fx, ly - 0.4))
    res = _score(rows)
    assert res['pass_xte'] is False and res['code'] == 1


def test_score_empty_rows_is_insufficient_not_crash():
    res = _score([])
    assert res['code'] == 2                          # 不崩潰、不 PASS


def test_adaptive_lookahead_preserves_v2_and_shrinks_at_low_speed():
    """V2 迴歸保證＋低速修正兩個關鍵點。"""
    # V2 模擬速度 0.35 m/s -> γ2.3×0.35=0.805 夾到上限 0.8 ＝原固定值，行為不變
    assert adaptive_lookahead(0.35) == pytest.approx(0.8)
    # V3 實車速度 0.2 m/s -> 0.46，切彎量幾何預估從 ~0.09m 降到 ~0.03m
    assert adaptive_lookahead(0.20) == pytest.approx(0.46)
    # 停車/極低速夾到下限，不會前視歸零造成震盪
    assert adaptive_lookahead(0.0) == pytest.approx(0.35)
    assert adaptive_lookahead(-0.5) == pytest.approx(0.8)   # 負速取絕對值


def test_score_xte_evaluable_flag():
    """leader 路徑沒有明顯長於跟車距離 -> xte_evaluable=False（僅標註，不改判定）。"""
    # 正常情境：leader 走 20m、跟車 1.2m -> 可評
    assert _score(_good_rows())['xte_evaluable'] is True
    # V3 煙霧測試情境：leader 只走 1.4m、跟車 1.2m -> 不可評，但 code 判定不受影響
    res = _score(_good_rows(), leader_length=1.4)
    assert res['xte_evaluable'] is False
    assert res['code'] in (0, 1)                     # 判定語義不變，只是多了標註


# --- 起點對齊（V3 任意起點）--------------------------------------------------

def test_yaw_from_quaternion_basic():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)      # 無旋轉
    # 繞 z 轉 90°：(x,y,z,w) = (0,0,sin45,cos45)
    s = math.sin(math.radians(45.0))
    assert yaw_from_quaternion(0.0, 0.0, s, s) == pytest.approx(math.pi / 2)


def test_align_v2_degenerates_to_translation():
    """V2 相容：leader 從原點 (0,0,0) 出發時，起點對齊退化成純 +gap0 平移。"""
    assert align_leader_point(3.0, 1.0, (0.0, 0.0, 0.0), 1.0) == pytest.approx((4.0, 1.0))
    assert align_leader_point(0.0, 0.0, (0.0, 0.0, 0.0), 1.0) == pytest.approx((1.0, 0.0))


def test_align_first_point_always_maps_to_gap0():
    """不論 leader odom 起點在哪、朝哪，起點本身永遠對齊到 follower 正前方 (gap0, 0)。"""
    for origin in [(0.0, 0.0, 0.0), (5.0, -3.0, 1.2), (-2.0, 7.0, -2.5)]:
        got = align_leader_point(origin[0], origin[1], origin, 1.0)
        assert got == pytest.approx((1.0, 0.0), abs=1e-9)


def test_align_arbitrary_start_normalizes_heading():
    """leader 從 (5,-3) 朝 +y(90°) 出發、沿車頭前進 2m，對齊後應是沿 +x 前進 2m。"""
    origin = (5.0, -3.0, math.pi / 2)
    # 朝車頭（+y）走 2m -> 世界座標 (5, -1)
    fx, fy = align_leader_point(5.0, -1.0, origin, 1.0)
    assert (fx, fy) == pytest.approx((3.0, 0.0), abs=1e-9)   # gap0 + 2m 沿 +x，橫向 0


def test_align_is_rigid_preserves_distances():
    """剛體變換：對齊前後，leader 軌跡上任兩點的距離不變（旋轉+平移保長）。"""
    origin = (5.0, -3.0, 0.8)
    a_w, b_w = (6.0, -2.0), (7.5, 0.5)
    a = align_leader_point(a_w[0], a_w[1], origin, 1.0)
    b = align_leader_point(b_w[0], b_w[1], origin, 1.0)
    d_world = math.hypot(a_w[0] - b_w[0], a_w[1] - b_w[1])
    d_aligned = math.hypot(a[0] - b[0], a[1] - b[1])
    assert d_aligned == pytest.approx(d_world, abs=1e-9)


# --- D9 V2V 鏈路延遲/丟包量測（LinkMonitor）----------------------------------

def test_seq_cov_index_is_valid_covariance_slot():
    """序號載體必須落在 6x6 covariance 的 0..35 之內。"""
    assert 0 <= V2V_SEQ_COV_INDEX <= 35


def test_link_no_loss_contiguous_sequence():
    """連續序號、固定 0.1s 延遲：零丟包、延遲統計正確。"""
    m = LinkMonitor()
    for seq in range(10):
        send_t = seq * 0.1
        recv_t = send_t + 0.1        # 固定 100ms 鏈路延遲
        m.record(send_t, recv_t, seq)
    s = m.summary()
    assert s['n'] == 10
    assert s['expected'] == 10       # 序號 0..9，跨度 10
    assert s['lost'] == 0
    assert s['loss_rate'] == pytest.approx(0.0)
    assert s['duplicated'] == 0 and s['reordered'] == 0
    assert s['delay_mean'] == pytest.approx(0.1)
    assert s['delay_rms'] == pytest.approx(0.1)
    assert s['delay_max'] == pytest.approx(0.1)


def test_link_counts_dropped_packets():
    """序號 0,1,2,5,6：中間缺 3,4 兩筆 -> lost=2、loss_rate=2/7。"""
    m = LinkMonitor()
    for seq in [0, 1, 2, 5, 6]:
        m.record(seq * 0.1, seq * 0.1 + 0.05, seq)
    s = m.summary()
    assert s['n'] == 5
    assert s['expected'] == 7        # 序號 0..6，跨度 7
    assert s['lost'] == 2            # 缺 3、4
    assert s['loss_rate'] == pytest.approx(2.0 / 7.0)


def test_link_delay_uses_recv_minus_send():
    """延遲純粹是 (收到 − 發送)，可為不同值 -> RMS 非平均。"""
    m = LinkMonitor()
    m.record(0.0, 0.03, 0)           # 30ms
    m.record(0.1, 0.14, 1)           # 40ms
    s = m.summary()
    assert s['delay_min'] == pytest.approx(0.03)
    assert s['delay_max'] == pytest.approx(0.04)
    assert s['delay_rms'] == pytest.approx(math.sqrt((0.03**2 + 0.04**2) / 2))


def test_link_reorder_does_not_pull_baseline_back():
    """晚到的舊序號（亂序）記為 reordered，且不可把 last_seq 拉回、害後續全誤判丟包。"""
    m = LinkMonitor()
    for seq in [0, 1, 2, 3]:
        m.record(seq * 0.1, seq * 0.1 + 0.05, seq)
    m.record(0.25, 0.30, 1)          # 序號 1 晚到（亂序）
    m.record(0.4, 0.45, 4)           # 接著正常的 4，不該被當成「從 1 跳到 4 -> 丟 2 筆」
    s = m.summary()
    assert s['reordered'] == 1
    assert s['lost'] == 0            # 4 是接在 3 之後的，沒有真丟包
    assert s['expected'] == 5        # 序號 0..4


def test_link_duplicate_sequence_counted():
    m = LinkMonitor()
    m.record(0.0, 0.05, 0)
    m.record(0.1, 0.15, 1)
    m.record(0.1, 0.16, 1)           # 重複序號 1
    s = m.summary()
    assert s['duplicated'] == 1
    assert s['lost'] == 0


def test_link_empty_summary_is_nan_not_crash():
    """一筆都沒收到：summary 不崩潰，延遲統計為 nan（不得當 0）。"""
    s = LinkMonitor().summary()
    assert s['n'] == 0 and s['lost'] == 0
    assert math.isnan(s['delay_rms'])
    assert math.isnan(s['loss_rate'])


def test_link_rows_shape_and_ms_conversion():
    """rows() 逐筆 (seq, send_t, recv_t, delay_ms, gap)，延遲已轉毫秒、gap 標新增丟包。"""
    m = LinkMonitor()
    m.record(0.0, 0.05, 0)
    m.record(0.3, 0.35, 3)           # 跳過 1,2 -> 這筆 gap=2
    rows = m.rows()
    assert len(rows) == 2
    assert rows[0] == pytest.approx((0, 0.0, 0.05, 50.0, 0))
    assert rows[1][0] == 3 and rows[1][3] == pytest.approx(50.0) and rows[1][4] == 2
