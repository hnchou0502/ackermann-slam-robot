"""拿 RM1 的**真實**姿態圖跑跨遍判定（零 ROS，Windows 直接 pytest）。

上面那支 `test_posegraph_math.py` 測的是人造情境；這支測的是 2026-08-29 從 car1 實機
撈下來的四組真圖。兩者缺一不可——人造測試釘語意，真資料防「語意訂對了、但現實長得
跟我想的不一樣」。

四組資料（`test/data/posegraph/`，來源見 Pi `~/cmpre/rm1_*`）：

  rm1_pass1              第一遍（baseline），8 節點
  rm1_pass2_lifelong     載入 pass1 續建（lifelong 節點）→ 應 PASS
  rm1_pass2_cont_async   載入 pass1 續建（async 節點）  → 應 PASS
  rm1_pass2_standalone   **對照組**：完全沒載入舊圖的獨立重建 → 必須擋下

對照組是本檔存在的主要理由：它的節點 ID 一樣從 0 編，光比對 ID 會冒出 5 條假跨遍邊。
任何改動只要讓 standalone 變成 PASS，就是把 RM2 的驗收證據弄假了。
"""
import json
import pathlib

import pytest

from stm32_bridge.posegraph_math import INVALID, PASS, analyse, resolve_edges

DATA = pathlib.Path(__file__).parent / "data" / "posegraph"


def load(name):
    d = json.loads((DATA / ("%s.json" % name)).read_text())
    nodes = {int(n[0]): (float(n[1]), float(n[2])) for n in d["nodes"]}
    return nodes, d["edge_points"]


def run(name):
    base, _ = load("rm1_pass1")
    nodes, edge_points = load(name)
    edges, problems = resolve_edges(nodes, edge_points)
    assert problems == [], "邊端點對不回節點：%s" % problems[:3]
    return analyse(base, nodes, edges)


def test_baseline_shape():
    """baseline 就是實機那 8 個節點；數字變了代表樣本被動過。"""
    nodes, _ = load("rm1_pass1")
    assert sorted(nodes) == [0, 1, 2, 3, 4, 5, 6, 7]
    assert nodes[0] == (0.0, 0.0)


@pytest.mark.parametrize("name,expect_pruned", [
    ("rm1_pass2_lifelong", [3, 4, 5, 6, 7]),   # lifelong 會裁舊節點
    ("rm1_pass2_cont_async", []),              # async 不裁、原封不動疊上去
])
def test_real_continuation_passes(name, expect_pruned):
    res = run(name)
    assert res["verdict"] == PASS, res["reason"]
    assert res["counts"]["cross"] >= 1
    assert res["checks"]["pruned"] == expect_pruned


def test_real_independent_rebuild_is_blocked():
    """對照組：獨立重建必須擋下來。這條變綠成 PASS ⇒ 整個 RM2 驗收作廢。"""
    res = run("rm1_pass2_standalone")
    assert res["verdict"] == INVALID
    assert res["counts"]["cross"] == 0


def test_id_only_comparison_would_have_false_passed():
    """記錄「為什麼不能只比 ID」——把位置檢查拿掉，對照組就會冒出假跨遍邊。

    這條不是在測產品碼，是在**釘住這個反例本身**：若哪天有人覺得位置比對多餘想拿掉，
    這裡的 5 就是現成的反證。
    """
    base, _ = load("rm1_pass1")
    nodes, edge_points = load("rm1_pass2_standalone")
    edges, _ = resolve_edges(nodes, edge_points)
    # 這組資料每個端點都只有一個候選，攤平後模擬「只比 ID」的做法
    flat = [(a[0], b[0]) for a, b in edges]
    id_only_cross = [(a, b) for a, b in flat if (a in base) != (b in base)]
    assert len(id_only_cross) == 5
    # 而真正的判定把它擋掉了
    assert analyse(base, nodes, edges)["verdict"] == INVALID
