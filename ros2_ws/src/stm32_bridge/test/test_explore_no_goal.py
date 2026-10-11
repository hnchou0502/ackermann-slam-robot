"""`classify_no_goal` 的語意測試（零 ROS，Windows 直接 pytest）。

**這支測試的存在理由**：2026-08-30 EX4 實車跑了三趟、三趟都提早收工，根因是
explorer 把 `choose_goal` 回傳 None 一律當成「探索完成」。那個 None 其實混了三種
完全不同的狀況，只有兩種該收工。

實測現場（run3，最能說明問題的一趟）：

    EXPLORATION DONE：no reachable frontier left (7 clusters, 3 blacklisted)

7 個叢集還在、只有 3 個試過失敗，**剩下 4 個連試都沒試**就宣告完成，
自主圖覆蓋率只有 39%（判準要 ≥70%）。

最後一條 `test_real_run2_map_is_not_ready` 是真實資料回歸：直接餵當時那張
98% 未知的地圖，斷言它必須判成 NOT_READY。這條變綠成「完成」就是把 bug 放回去。
"""
import pathlib

import pytest

from stm32_bridge.explore_math import (
    ALL_BLACKLISTED, NOT_READY, NO_FRONTIER,
    Blacklist, Grid, choose_goal, classify_no_goal, extract_frontiers,
    goal_for_frontier,
)

CH = {'#': 100, '.': 0, '?': -1}
DATA = pathlib.Path(__file__).parent / "data" / "explore"


def grid_from(rows, resolution=0.05, origin=(0.0, 0.0)):
    """字串列表 → Grid。第一個字串是最上面一列（y 最大）。慣例同 test_explore_math。"""
    h, w = len(rows), len(rows[0])
    data = []
    for j in range(h):
        data.extend(CH[c] for c in rows[h - 1 - j])
    return Grid(data, w, h, resolution, origin[0], origin[1])


# 解析度 0.05 m ⇒ 退避 0.5 m ＝ 10 格、淨空 0.25 m ＝ 5 格、邊界叢集門檻 6 格。
# 下面兩張圖是照這些數字精算的，改門檻就要重畫。

W = 30

# 已知空地 20 格寬 ⇒ 從邊界往回退 10 格還在空地裡，且四周 5 格內無障礙 ⇒ 產得出目標
ROOMY = (['#' * W]
         + ['#' + '.' * 20 + '?' * 9] * 15
         + ['#' * W])

# 已知空地只有 4 格寬 ⇒ 邊界有 10 格（過門檻），但退 10 格會退到牆外 ⇒ 產不出目標
SLIVER = (['#' * W]
          + ['#' + '.' * 4 + '?' * 25] * 10
          + ['#' * W])

# 上半可產出目標、下半產不出來的混合圖
MIXED = ROOMY + SLIVER

ALL_KNOWN = [
    '##########',
    '#........#',
    '#........#',
    '##########',
]


def test_no_frontier_at_all_is_done():
    """整張圖都已知＝真的探索完了，這時收工是對的。"""
    g = grid_from(ALL_KNOWN)
    assert extract_frontiers(g) == []
    assert classify_no_goal(g, extract_frontiers(g)) == NO_FRONTIER


def test_sliver_map_is_not_ready_not_done():
    """邊界還在、但已知區太窄退不出立足點 ⇒ NOT_READY，**不准當成探索完成**。

    這正是 EX4 run2 的情境：車還沒動、地圖幾乎全未知，叢集都產不出目標。
    """
    g = grid_from(SLIVER)
    fr = extract_frontiers(g)
    assert fr, '這張圖應該要有邊界叢集，否則測的不是想測的東西'
    assert all(goal_for_frontier(g, f) is None for f in fr), '前提：全部產不出目標'
    assert classify_no_goal(g, fr) == NOT_READY


def test_all_formable_goals_blacklisted_is_done():
    """每個邊界都產得出目標、卻全被黑名單擋下＝試過且失敗，這時收工是對的。"""
    g = grid_from(ROOMY)
    fr = extract_frontiers(g)
    assert fr
    bl = Blacklist()
    for f in fr:
        goal = goal_for_frontier(g, f)
        assert goal is not None, '前提：這張圖產得出目標'
        bl.add(goal[0], goal[1])
    assert choose_goal(g, fr, (0.0, 0.0), bl) == (None, None)
    assert classify_no_goal(g, fr, bl) == ALL_BLACKLISTED


def test_mixed_case_prefers_not_ready():
    """有些邊界產得出目標但被黑名單、有些根本產不出來 ⇒ 一律 NOT_READY。

    產不出來的那些可能隨地圖長大變得可用；此時宣告完成就是重演 EX4 的 bug。
    寧可等到總時限，也不要假的「探索完成」。
    """
    g = grid_from(MIXED)
    fr = extract_frontiers(g)
    assert fr
    bl = Blacklist()
    for f in fr:
        goal = goal_for_frontier(g, f)
        if goal is not None:
            bl.add(goal[0], goal[1])
    assert classify_no_goal(g, fr, bl) == NOT_READY


def test_verdicts_are_distinct_strings():
    """三個判決不可互相相等——混在一起就是這個 bug 的來源。"""
    assert len({NO_FRONTIER, ALL_BLACKLISTED, NOT_READY}) == 3


# --- 真實資料回歸：EX4 run2 當時那張地圖 -------------------------------

def _load_run2_grid():
    from PIL import Image
    im = Image.open(DATA / "run2_map.pgm").convert("L")
    w, h = im.size
    px = list(im.getdata())
    data = []
    for r in range(h - 1, -1, -1):          # pgm row 0 在上；Grid 的 j=0 在 y 最小
        for c in range(w):
            v = px[r * w + c]
            data.append(100 if v <= 99 else (0 if v >= 206 else -1))
    ox = oy = 0.0
    for line in open(DATA / "run2_map.yaml"):
        if line.startswith("origin"):
            ox, oy = [float(x) for x in line.split("[")[1].split("]")[0].split(",")[:2]]
    return Grid(data, w, h, 0.05, ox, oy)


def test_real_run2_map_is_not_ready():
    """EX4 run2 的真實地圖：98% 未知、4 個叢集全產不出目標 ⇒ 必須是 NOT_READY。

    修好之前，explorer 對這張圖回「no reachable frontier left」並在 23 秒收工，
    車一步都沒走。這條變綠成任何一種「完成」，就是把 bug 放回去。
    """
    pytest.importorskip("PIL")
    g = _load_run2_grid()
    known = sum(1 for v in g.data if v >= 0)
    assert known / len(g.data) < 0.05, '前提：這張圖應該幾乎全是未知'

    fr = extract_frontiers(g)
    assert len(fr) == 4, '實測當時就是 4 個叢集'
    assert choose_goal(g, fr, (0.0, 0.0)) == (None, None), '實測：一個合法目標都產不出來'
    assert classify_no_goal(g, fr) == NOT_READY


# --- 接線測試：光有純函式不夠，要確認 explorer 真的用了它 -----------------

def test_explorer_actually_uses_the_classifier():
    """`frontier_explorer` 的「goal is None」分支必須先分類再決定收不收工。

    **為什麼需要這條**：上面的測試全部只測純函式。若有人把 explorer 改回
    `if goal is None: self._finish(...)`，純函式測試照樣全綠，bug 卻回來了——
    2026-08-30 EX4 三趟全部提早收工就是死在這一行。這條盯的是接線，不是邏輯。
    """
    src = (pathlib.Path(__file__).parent.parent
           / "stm32_bridge" / "frontier_explorer.py").read_text(encoding="utf-8")
    assert "classify_no_goal" in src, 'explorer 沒有 import／呼叫 classify_no_goal'

    i = src.index("if goal is None:")
    j = src.index("self.current_goal = goal")
    branch = src[i:j]
    assert "classify_no_goal" in branch, \
        '「goal is None」分支沒有先分類就決定收工——這正是那個 bug'
    assert "NOT_READY" in branch, '分支沒有處理 NOT_READY（暫時產不出目標）'
    assert branch.index("classify_no_goal") < branch.index("_finish"), \
        '必須先分類、再決定要不要 _finish'
