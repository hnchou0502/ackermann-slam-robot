"""posegraph_math 的單元測試（零 ROS，Windows 直接 pytest）。

RM2 驗收條件①的判定邏輯。重點不是「跑得動」，而是**釘住語意**：
每一條都對應一種「看起來像通過、其實不算數」的情境，拿掉對應的防線就必須變紅。

最重要的兩條是 `test_independent_rebuild_is_invalid_not_pass`（對照組）與
`test_lifelong_pruned_baseline_still_passes`（不得把合格的續建誤判成 INVALID）——
兩者都是 2026-08-29 拿 car1 的真實資料跑出來才發現的，數值直接取自實測。
"""
import pytest

from stm32_bridge.posegraph_math import (
    BASELINE_POSE_TOL_M,
    FAIL,
    INVALID,
    MAX_LISTED,
    PASS,
    analyse,
    dedupe_edges,
    format_report,
    normalise_edge,
    resolve_edges,
)

# 取自 2026-08-29 car1 實測（rm1_pass1 的前三個節點），讓測試資料與現場一致
P0, P1, P2 = (0.0, 0.0), (0.875, -0.023), (1.309, -0.148)


def line(a, b):
    return (a[0], a[1], b[0], b[1])


# --- 邊的正規化與去重 -------------------------------------------------

def test_edge_is_undirected():
    # 端點正規化成「候選 tuple」，方向不影響結果
    assert normalise_edge(7, 3) == ((3,), (7,))
    assert normalise_edge(3, 7) == ((3,), (7,))
    assert normalise_edge((7, 9), 3) == ((3,), (7, 9))


def test_dedupe_drops_reversed_duplicates_and_self_loops():
    assert dedupe_edges([(2, 5), (5, 2), (4, 4), (1, 2)]) == [((1,), (2,)), ((2,), (5,))]


def test_reversed_edge_counted_once_not_twice():
    """同一條跨遍邊用相反方向各給一次，不得算成兩條證據。"""
    res = analyse({0: P0, 1: P1}, {0: P0, 1: P1, 8: (2.0, 0.0)}, [(1, 8), (8, 1)])
    assert res["verdict"] == PASS
    assert res["counts"]["cross"] == 1


# --- 線段端點對回節點 -------------------------------------------------

def test_resolve_edges_exact_match():
    nodes = {0: P0, 1: P1, 2: P2}
    edges, problems = resolve_edges(nodes, [line(P0, P1), line(P1, P2)])
    assert problems == []
    # 每個端點都只有一個候選＝明確
    assert edges == [((0,), (1,)), ((1,), (2,))]


def test_resolve_edges_flags_unmatched_endpoint():
    """端點落在沒有節點的地方＝圖抓得不完整，必須回報而不是硬湊最近的。"""
    nodes = {0: P0, 1: P1}
    edges, problems = resolve_edges(nodes, [line(P0, (5.0, 5.0))])
    assert edges == []
    assert problems and "對不上任何節點" in problems[0]


def test_resolve_edges_reports_all_candidates_when_ambiguous():
    """兩個節點幾乎重疊時，把候選全部列出來，不准任選一個。

    任選一個＝靜默的錯誤分類。列出候選後，由 analyse 決定這條邊還能不能當證據。
    """
    nodes = {0: P0, 1: (1.000, 0.0), 2: (1.002, 0.0)}
    edges, problems = resolve_edges(nodes, [line(P0, (1.001, 0.0))])
    assert problems == []
    assert edges == [((0,), (1, 2))]   # 第二端分不出是 1 還是 2


# --- 正例 -------------------------------------------------------------

def test_cross_pass_constraint_found():
    base = {0: P0, 1: P1, 2: P2}
    final = {**base, 8: (2.0, -0.2), 9: (2.5, -0.3)}
    res = analyse(base, final, [(0, 1), (1, 2), (2, 8), (8, 9)])
    assert res["verdict"] == PASS
    assert res["counts"] == {"pass1": 2, "pass2": 1, "cross": 1, "undecidable": 0}
    assert res["cross_edges"] == [((2,), (8,))]


def test_lifelong_pruned_baseline_still_passes():
    """lifelong 會裁掉舊節點——8 個只剩 3 個是實測到的正常行為，不得判 INVALID。

    2026-08-29 實測：載入 8 節點的第一遍圖續建後只剩 ID 0,1,2，其餘被裁。
    早期版本要求「第一遍節點必須全在」，會把 RM1 選定的方案判成資料無效。
    """
    base = {i: (0.5 * i, 0.0) for i in range(8)}
    final = {0: base[0], 1: base[1], 2: base[2]}
    final.update({20 + i: (4.0 + 0.5 * i, 1.0) for i in range(5)})
    res = analyse(base, final, [(0, 1), (1, 2), (2, 20), (20, 21)])
    assert res["verdict"] == PASS
    assert res["checks"]["pruned"] == [3, 4, 5, 6, 7]
    assert "裁掉" in format_report(res)


# --- 對照組：本檔最重要的一條 -------------------------------------------

def test_independent_rebuild_is_invalid_not_pass():
    """獨立重建的第二遍：ID 一樣從 0 編，但節點是完全不同的節點 ⇒ 不得 PASS。

    這是 RM1 的 `rm1_pass2_standalone` 情境。2026-08-29 實測：光比對 ID 會冒出
    **5 條假跨遍邊**；同 ID 節點的位置差平均 0.754 m、最大 1.227 m。
    位置差的門檻就是為了擋這個。工具若對這組資料回 PASS，整個 RM2 驗收作廢。
    """
    base = {0: P0, 1: P1, 2: P2}
    # 同樣的 ID，但這是另一趟自己走出來的軌跡（數值取自實測 standalone 的前三點）
    final = {0: (0.0, 0.0), 1: (0.776, -0.294), 2: (0.612, -0.704), 8: (1.5, -1.0)}
    res = analyse(base, final, [(0, 1), (1, 2), (2, 8)])
    assert res["verdict"] == INVALID
    assert "沒載入" in res["reason"]


def test_pose_tolerance_boundary():
    """位置差剛好在容忍值內要放行、超過要擋——門檻不能只是裝飾。"""
    base = {0: P0, 1: (1.0, 0.0), 2: (2.0, 0.0)}
    inside = {**base, 1: (1.0 + BASELINE_POSE_TOL_M * 0.5, 0.0), 8: (3.0, 0.0)}
    assert analyse(base, inside, [(2, 8)])["verdict"] == PASS
    outside = {**base, 1: (1.0 + BASELINE_POSE_TOL_M * 2, 0.0), 8: (3.0, 0.0)}
    assert analyse(base, outside, [(2, 8)])["verdict"] == INVALID


# --- 其餘 FAIL／INVALID ------------------------------------------------

def test_no_cross_edge_is_fail():
    base = {0: P0, 1: P1, 2: P2}
    final = {**base, 8: (5.0, 5.0), 9: (5.5, 5.0)}
    res = analyse(base, final, [(0, 1), (1, 2), (8, 9)])
    assert res["verdict"] == FAIL
    assert "各走各的" in res["reason"]


def test_no_new_nodes_is_fail_with_distinct_reason():
    """第二遍一個新節點都沒有＝車沒走。要與「走了但沒接上」分開講，兩者下一步不同。"""
    base = {0: P0, 1: P1, 2: P2}
    res = analyse(base, {**base}, [(0, 1), (1, 2)])
    assert res["verdict"] == FAIL
    assert res["n_pass2"] == 0
    assert "沒有產生任何新節點" in res["reason"]


def test_only_origin_node_surviving_is_invalid():
    """只剩原點節點時位置比對沒有鑑別力——每張圖的節點 0 都在 (0,0)。"""
    base = {0: P0, 1: P1, 2: P2}
    final = {0: P0, 8: (1.0, 1.0), 9: (1.5, 1.0)}
    res = analyse(base, final, [(0, 8), (8, 9)])
    assert res["verdict"] == INVALID
    assert "鑑別力" in res["reason"]


def test_empty_baseline_is_invalid():
    assert analyse({}, {1: P1}, [])["verdict"] == INVALID


def test_empty_final_is_invalid():
    assert analyse({0: P0, 1: P1}, {}, [])["verdict"] == INVALID


def test_dangling_edge_endpoint_is_invalid():
    base = {0: P0, 1: P1}
    final = {**base, 8: (2.0, 0.0)}
    res = analyse(base, final, [(1, 8), (8, 99)])
    assert res["verdict"] == INVALID
    assert "不在節點集合裡" in res["reason"]


def test_invalid_takes_priority_over_cross_evidence():
    """既有跨遍邊、又有資料瑕疵時必須報 INVALID——不得因為「反正有證據」就放行。"""
    base = {0: P0, 1: P1}
    final = {**base, 8: (2.0, 0.0)}
    assert analyse(base, final, [(1, 8), (8, 77)])["verdict"] == INVALID


# --- 報表 -------------------------------------------------------------

def test_report_contains_verdict_and_cross_count():
    base = {0: P0, 1: P1, 2: P2}
    text = format_report(analyse(base, {**base, 8: (2.0, 0.0)}, [(2, 8)]))
    assert "PASS" in text and "跨遍 1" in text


def test_cross_edge_list_is_capped_but_count_is_not():
    """跨遍邊很多時清單截斷，但計數必須完整（判定依據是計數不是清單）。"""
    n = MAX_LISTED + 5
    base = {i: (0.5 * (i + 1), 0.0) for i in range(n)}
    final = dict(base)
    final.update({1000 + i: (0.5 * (i + 1), 1.0) for i in range(n)})
    res = analyse(base, final, [(i, 1000 + i) for i in range(n)])
    assert res["verdict"] == PASS
    assert res["counts"]["cross"] == n
    assert len(res["cross_edges"]) == MAX_LISTED


@pytest.mark.parametrize("bad", [None, "abc"])
def test_non_numeric_node_ids_raise(bad):
    """節點 ID 不是數字就直接炸，不要靜默吞掉——那會變成無聲的錯誤分類。"""
    with pytest.raises((TypeError, ValueError)):
        analyse({0: P0, 1: P1}, {0: P0, 1: P1, bad: (2.0, 0.0)}, [(0, 1)])


# --- 端點歧義：車回到原地重開時的實況（2026-08-30 實測 6 mm）-------------

def test_ambiguous_endpoint_spanning_passes_is_not_evidence():
    """候選一個屬第一遍、一個屬第二遍 ⇒ 這條邊無法判定，**絕不可算成跨遍證據**。

    實況：第二遍從第一遍的起點原地重開，新節點長在舊節點旁 6 mm 處。
    若把這種邊算成跨遍，等於「車停在原地不動也能拿到證據」。
    """
    base = {0: P0, 1: P1, 2: P2}
    final = {**base, 17: (0.0, -0.006), 18: (3.0, 1.0)}   # 17 貼著 baseline 的 0
    res = analyse(base, final, [((0, 17), (18,))])
    assert res["counts"]["undecidable"] == 1
    assert res["counts"]["cross"] == 0
    assert res["verdict"] == FAIL
    assert "無法判定" in format_report(res)


def test_ambiguous_endpoint_within_same_pass_still_counts():
    """候選雖然有兩個，但兩個都屬第一遍 ⇒ 這一端的歸屬其實是確定的，照常採計。

    模糊只該在「分不出屬於哪一遍」時才讓證據失效，不該一律作廢。
    """
    base = {0: P0, 1: (1.000, 0.0), 2: (1.002, 0.0)}
    final = {**base, 17: (3.0, 1.0)}
    res = analyse(base, final, [((1, 2), (17,))])
    assert res["counts"]["cross"] == 1
    assert res["counts"]["undecidable"] == 0
    assert res["verdict"] == PASS
