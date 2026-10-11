"""map_compare_math 的單元測試（零 ROS，Windows 直接 pytest）。

RM2 驗收條件②的判定邏輯。每一條對應一種「算出來的數字看似合理、其實是假的」的情境。
"""
import pytest

from stm32_bridge.map_compare_math import (
    FREE, OCC, UNKNOWN,
    cell_offset, classify, inner_wall_cells, known_mask,
    masked_compare, to_cell, to_world, verdict,
)

O, F, U = 0, 254, 205   # 障礙／空地／未知


def grid(rows_str):
    """用字元畫地圖：'#'=障礙 '.'=空地 '?'=未知。"""
    m = {"#": O, ".": F, "?": U}
    return [[m[ch] for ch in row] for row in rows_str.strip().splitlines()]


# --- 值語意 -----------------------------------------------------------

@pytest.mark.parametrize("v,expect", [(0, OCC), (99, OCC), (100, UNKNOWN),
                                      (205, UNKNOWN), (199, UNKNOWN),
                                      (206, FREE), (254, FREE)])
def test_classify_boundaries(v, expect):
    assert classify(v) == expect


# --- 座標換算 ---------------------------------------------------------

def test_world_cell_round_trip():
    res, origin, h = 0.05, (-7.805, -2.425), 91
    for row, col in [(0, 0), (45, 120), (90, 238)]:
        x, y = to_world(row, col, res, origin, h)
        assert to_cell(x, y, res, origin, h) == (row, col)


def test_row_zero_is_top_of_map():
    """影像 row 0 對應世界座標 y 最大處。搞反的話整張圖會上下顛倒還不報錯。"""
    res, origin, h = 0.05, (0.0, 0.0), 10
    _, y_top = to_world(0, 0, res, origin, h)
    _, y_bottom = to_world(9, 0, res, origin, h)
    assert y_top > y_bottom


def test_cell_offset_reports_subcell_residual():
    """次格錯位必須被回報——否則呼叫端不知道該不該容忍。

    RM2 實測：兩張圖 y 原點差 0.685 m ÷ 0.05 = 13.7 格，殘留 0.7 格。
    """
    drow, dcol, residual = cell_offset(
        0.05, (-7.805, -2.425), 91, (-7.805, -3.110), 134)
    assert dcol == 0
    assert drow == 29                       # (134-91) - 13.7 → 29
    assert residual == pytest.approx(0.3)   # 13.7 格四捨五入成 14，殘留 0.3 格
    assert residual > 0.25                  # 非整數格 ⇒ 嚴格逐格比對會產生假衝突


def test_cell_offset_integer_case_has_no_residual():
    _, _, residual = cell_offset(0.05, (0.0, 0.0), 10, (0.0, -0.5), 20)
    assert residual == pytest.approx(0.0)


# --- 遮罩比對 ---------------------------------------------------------

IDENTICAL = """
?????
?...?
?.#.?
?...?
?????
"""


def test_identical_maps_have_zero_conflict():
    g = grid(IDENTICAL)
    r = masked_compare(g, g, 0, 0)
    assert r["occ_to_free"] == 0 and r["free_to_occ"] == 0
    assert r["known_to_unknown"] == 0
    assert r["agree"] == r["masked_cells"] == 9   # 中間 3x3 是已知的


def test_unknown_cells_are_outside_the_comparison():
    """第一遍不知道的格子不該進入比較——否則第二遍掃到新東西會被算成『衝突』。"""
    a = grid(IDENTICAL)
    b = grid("""
#####
#...#
#.#.#
#...#
#####
""")
    r = masked_compare(a, b, 0, 0)
    assert r["masked_cells"] == 9      # 只比 a 已知的那 9 格
    assert r["occ_to_free"] == 0 and r["free_to_occ"] == 0


def test_wall_turned_into_free_is_counted():
    a = grid(IDENTICAL)
    b = grid("""
?????
?...?
?...?
?...?
?????
""")
    r = masked_compare(a, b, 0, 0, tolerance=0)
    assert r["occ_to_free"] == 1


def test_free_turned_into_wall_is_counted():
    a = grid(IDENTICAL)
    b = grid("""
?????
?.#.?
?.#.?
?...?
?????
""")
    r = masked_compare(a, b, 0, 0, tolerance=0)
    assert r["free_to_occ"] == 1


def test_known_becoming_unknown_is_tracked_separately():
    """資訊流失與畫錯是兩回事，成因不同，不可混為一談。"""
    a = grid(IDENTICAL)
    b = grid("""
?????
?...?
?.?.?
?...?
?????
""")
    r = masked_compare(a, b, 0, 0, tolerance=0)
    assert r["known_to_unknown"] == 1
    assert r["occ_to_free"] == 0


def test_one_cell_shift_is_absorbed_by_tolerance():
    """整張圖平移一格＝對齊誤差，不是劣化。容忍值就是為了這個。"""
    a = grid("""
?????
?...?
?.#.?
?...?
?????
""")
    b = grid("""
?????
?.#.?
?...?
?...?
?????
""")
    assert masked_compare(a, b, 0, 0, tolerance=1)["occ_to_free"] == 0
    # 容忍值設 0 就抓得到——證明這個參數真的有作用，不是裝飾
    assert masked_compare(a, b, 0, 0, tolerance=0)["occ_to_free"] == 1


# --- 假牆 -------------------------------------------------------------

def test_wall_touching_border_is_not_fake():
    g = grid("""
#####
#...#
#...#
#####
""")
    assert inner_wall_cells(g)["inner_cells"] == 0


def test_floating_wall_block_is_fake():
    g = grid("""
#######
#.....#
#.###.#
#.....#
#######
""")
    r = inner_wall_cells(g)
    assert r["inner_components"] == 1
    assert r["inner_cells"] == 3


def test_connectivity_is_computed_on_full_map_not_inside_mask():
    """連通性必須在整張圖上算。

    只在 mask 內算連通，會把「本來連到外牆、但外牆在 mask 外」的牆段誤判成孤立假牆——
    憑空製造劣化。這條就是釘住這件事。
    """
    g = grid("""
#######
#.....#
#.....#
##....#
#######
""")
    # mask 只涵蓋中間，刻意排除左下那段與外牆相連的牆
    mask = {(r, c) for r in range(1, 4) for c in range(1, 6)}
    assert inner_wall_cells(g, mask=mask)["inner_cells"] == 0


def test_mask_limits_which_fake_walls_are_counted():
    # ⚠ 每塊假牆要 >= MIN_COMPONENT_CELLS(3) 格，否則會被雜訊門檻濾掉
    g = grid("""
#########
#.......#
#.###...#
#.......#
#...###.#
#.......#
#########
""")
    full = inner_wall_cells(g)
    assert full["inner_components"] == 2
    left_only = {(r, c) for r in range(1, 4) for c in range(1, 4)}
    assert inner_wall_cells(g, mask=left_only)["inner_components"] == 1


def test_known_mask_excludes_unknown():
    g = grid("""
??.
.#?
""")
    assert known_mask(g) == {(0, 2), (1, 0), (1, 1)}


# --- 判準 -------------------------------------------------------------

def test_verdict_fails_when_fake_walls_increase():
    v = verdict({"inner_cells": 3, "inner_components": 1},
                {"inner_cells": 9, "inner_components": 2},
                {"occ_to_free": 0, "free_to_occ": 0, "known_to_unknown": 0})
    assert v["pass"] is False and "劣化" in v["reason"]


def test_verdict_passes_when_fake_walls_do_not_increase():
    v = verdict({"inner_cells": 5, "inner_components": 1},
                {"inner_cells": 5, "inner_components": 1},
                {"occ_to_free": 2, "free_to_occ": 1, "known_to_unknown": 4})
    assert v["pass"] is True
    assert v["conflicts"] == 3
    assert v["known_to_unknown"] == 4
