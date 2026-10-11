"""merge_math（雙車合圖純數學）單元測試——Codex F4：核心改動須有 ROS-free 覆蓋。

全部在 Windows 裸 pytest 可跑（只需 numpy）。
涵蓋：TF 慣例、OccupancyGrid 方向（無 y 翻轉）、已知變換還原、
直牆歧義拒絕、空曠衝突偵測、融合規則與邊界。
"""
import math

import numpy as np
import pytest

from stm32_bridge import merge_math as mm


# ---------- 幾何產生器 ----------

def rect_walls(w=3.0, h=2.0, step=0.05):
    """矩形房間外牆點集（原點在左下角）。"""
    xs = np.arange(0.0, w + 1e-9, step)
    ys = np.arange(0.0, h + 1e-9, step)
    pts = []
    for x in xs:
        pts.append((x, 0.0))
        pts.append((x, h))
    for y in ys:
        pts.append((0.0, y))
        pts.append((w, y))
    return np.array(pts)


def l_shape_walls(step=0.05):
    """L 形（打破矩形的 180° 對稱，讓解唯一）。"""
    rect = rect_walls(3.0, 2.0, step)
    notch_x = np.arange(1.5, 3.0 + 1e-9, step)
    notch_y = np.arange(1.0, 2.0 + 1e-9, step)
    pts = [p for p in rect if not (p[0] > 1.5 + 1e-6 and p[1] > 1.0 + 1e-6)]
    for x in notch_x:
        pts.append((x, 1.0))
    for y in notch_y:
        pts.append((1.5, y))
    return np.array(pts)


# ---------- TF 慣例 ----------

def test_transform_pts_convention():
    # 與 merge_maps_offline 的公式逐字一致：xa = tx + c·xb − s·yb
    pts = np.array([[1.0, 0.0], [0.0, 2.0]])
    out = mm.transform_pts(pts, tx=10.0, ty=20.0, yaw=math.pi / 2)
    # (1,0) 轉 90° → (0,1)，平移 → (10, 21)
    assert out[0] == pytest.approx([10.0, 21.0], abs=1e-9)
    # (0,2) 轉 90° → (-2,0)，平移 → (8, 20)
    assert out[1] == pytest.approx([8.0, 20.0], abs=1e-9)


def test_walls_from_grid_no_y_flip():
    # OccupancyGrid：row 0 在 origin 側（y 最小），與 PGM（row 0 在頂）相反。
    # 3 寬 × 2 高，唯一牆格在 row1, col2 → 世界座標 y 應是「大」的那排。
    data = [0, 0, 0,
            0, 0, 100]
    pts = mm.walls_from_grid(data, width=3, height=2, res=1.0, ox=10.0, oy=20.0)
    assert pts.shape == (1, 2)
    assert pts[0] == pytest.approx([10.0 + 2.5, 20.0 + 1.5])


def test_free_from_grid_excludes_unknown():
    data = [-1, 0, 100, 30]   # 未知 / 空曠 / 牆 / 中間值(非明確空曠)
    pts = mm.free_from_grid(data, width=4, height=1, res=1.0, ox=0.0, oy=0.0)
    assert pts.shape == (1, 2)
    assert pts[0] == pytest.approx([1.5, 0.5])


# ---------- 對齊：已知變換還原 ----------

def test_align_recovers_known_transform():
    res = 0.05
    a = l_shape_walls(res)
    true_yaw = math.radians(33.0)
    true_tx, true_ty = 0.75, 0.25
    # B 圖＝把 A 系點反向搬進 B 系（B→A 用 true_tf 時應還原）
    c, s = math.cos(-true_yaw), math.sin(-true_yaw)
    shifted = a - [true_tx, true_ty]
    b = np.stack([c * shifted[:, 0] - s * shifted[:, 1],
                  s * shifted[:, 0] + c * shifted[:, 1]], axis=1)

    out = mm.align(a, b, res, coarse_deg=2.0)
    assert out is not None
    assert not out["saturated"]
    assert out["score"] > 0.7
    # 好幾何的歧義比要低於節點門檻 0.80（與直牆案例分離；門檻校準見 map_merge_live）
    assert out["ambiguity"] < 0.8
    err_deg = abs((math.degrees(out["yaw"] - true_yaw) + 180.0) % 360.0 - 180.0)
    assert err_deg < 1.0, f"yaw 誤差 {err_deg:.2f}°"
    # 平移誤差 < 2 格（角度離散化會帶進 cm 級誤差）
    assert abs(out["tx"] - true_tx) < 2 * res
    assert abs(out["ty"] - true_ty) < 2 * res
    # 驗證回代：B 牆經還原變換後應與 A 牆幾乎重合
    tb = mm.transform_pts(b, out["tx"], out["ty"], out["yaw"])
    d = np.sqrt(((tb - a) ** 2).sum(axis=1))
    assert np.median(d) < 2 * res


def test_align_straight_wall_is_ambiguous():
    # 單一直牆：0° 與 180° 皆可（近乎）完美重合 → 歧義比必須超過節點拒絕門檻 0.80。
    # 實測值 ≈0.896（180° 峰因柵格化浮點抖動略低於 1.0——這正是門檻從 0.90 調到
    # 0.80 的原因：0.90 會放行這個真歧義案例）。
    res = 0.05
    xs = np.arange(0.0, 3.0 + 1e-9, res)
    line = np.stack([xs, np.zeros_like(xs)], axis=1)
    out = mm.align(line, line.copy(), res, coarse_deg=2.0)
    assert out is not None
    assert out["ambiguity"] > 0.8, f"直牆應為歧義解，ambiguity={out['ambiguity']:.2f}"


# ---------- 空曠衝突 ----------

def test_free_conflict_low_when_aligned_high_when_shifted():
    res = 0.05
    walls = rect_walls(3.0, 2.0, res)
    # A 的空曠＝房間內部
    gx, gy = np.meshgrid(np.arange(0.2, 2.8, res), np.arange(0.2, 1.8, res))
    free = np.stack([gx.ravel(), gy.ravel()], axis=1)
    # 對齊：B 牆＝A 牆 → 衝突≈0
    ok = mm.free_conflict_ratio(free, walls, 0.0, 0.0, 0.0, res)
    assert ok < 0.05
    # 錯位 1m：B 牆大量落進 A 內部空曠 → 衝突明顯
    bad = mm.free_conflict_ratio(free, walls, 1.0, 0.5, 0.0, res)
    assert bad > 0.3, f"錯位衝突應該高，got {bad:.2f}"


# ---------- 融合 ----------

def _grid(data, w, h, res=1.0, ox=0.0, oy=0.0):
    return dict(data=data, width=w, height=h, res=res, ox=ox, oy=oy)


def test_fuse_rules_wall_beats_free_beats_unknown():
    # A：左格牆、右格未知；B（恆等變換、同範圍）：左格空曠、右格空曠
    a = _grid([100, -1], 2, 1)
    b = _grid([0, 0], 2, 1)
    out, minx, miny = mm.fuse_grids(a, b, 0.0, 0.0, 0.0)
    assert (minx, miny) == (0.0, 0.0)
    assert out.shape == (1, 2)
    assert out[0, 0] == 100     # 牆優先於空曠
    assert out[0, 1] == 0       # 空曠優先於未知


def test_fuse_extent_covers_transformed_b():
    # B 平移 (5,0) 後，輸出範圍必須涵蓋 B 的新位置
    a = _grid([100], 1, 1)
    b = _grid([100], 1, 1)
    out, minx, miny = mm.fuse_grids(a, b, 5.0, 0.0, 0.0)
    H, W = out.shape
    assert minx <= 0.0 and minx + W * 1.0 >= 6.0
    # 兩面牆都要出現在輸出裡
    assert (out == 100).sum() == 2


# ---------- 決策政策（Codex R4-F2：政策本身必須被測試咬住） ----------

def _result(score=0.6, ambiguity=0.3, saturated=False, kind="angle"):
    return {"score": score, "yaw": 0.0, "tx": 0.0, "ty": 0.0,
            "runner_score": score * ambiguity, "runner_yaw": 0.5,
            "runner_kind": kind, "ambiguity": ambiguity, "saturated": saturated}


def test_accept_all_reject_branches_and_pass():
    assert mm.accept(None, 0.0) == (False, "no_solution")
    assert mm.accept(_result(saturated=True), 0.0) == (False, "saturated")
    assert mm.accept(_result(score=0.1), 0.0) == (False, "low_score")
    ok, reason = mm.accept(_result(ambiguity=0.95, kind="shift"), 0.0)
    assert (ok, reason) == (False, "ambiguous_shift")
    assert mm.accept(_result(), 0.9) == (False, "free_conflict")
    assert mm.accept(_result(), 0.1) == (True, "ok")


def test_accept_thresholds_are_calibrated_values():
    # 鎖住實資料校準值（2026-07-21 四案例，見 merge_math 常數區與
    # docs/design/merge_guard_calibration.md）。改門檻＝改政策，必須連這裡一起改，
    # 並附新的校準證據。
    assert mm.DEFAULT_MIN_SCORE == 0.30
    assert mm.DEFAULT_MAX_AMBIGUITY == 0.80
    assert mm.DEFAULT_MAX_CONFLICT == 0.25
    # 邊界方向：正解案例（歧義 0.53、衝突 0.198）必須通過；
    # 真歧義（0.896）與錯解衝突（0.330）必須被擋
    assert mm.accept(_result(ambiguity=0.53), 0.198)[0] is True
    assert mm.accept(_result(ambiguity=0.896), 0.0)[0] is False
    assert mm.accept(_result(), 0.330)[0] is False


# ---------- R4-F1：同角度、不同平移的歧義（相同房間/重複走廊） ----------

def test_two_identical_rooms_is_shift_ambiguous():
    # A＝兩個一模一樣的房間並排（相距 4m）；B＝其中一個房間。
    # 正確行為：偵測到同 yaw 有兩個等高平移峰 → 歧義比高 → accept 拒絕。
    # 修 R4-F1 前的行為：argmax 任選一間發布，歧義比低（假自信）。
    res = 0.05
    room = rect_walls(2.0, 1.5, res)
    a = np.vstack([room, room + [4.0, 0.0]])
    b = room.copy()
    out = mm.align(a, b, res, coarse_deg=2.0)
    assert out is not None
    assert out["ambiguity"] > mm.DEFAULT_MAX_AMBIGUITY, \
        f"兩間相同房間應為平移歧義，ambiguity={out['ambiguity']:.2f}"
    assert not mm.accept(out, 0.0)[0]


def _twin_block_images(off_y, off_x, size=3, canvas=64):
    """A＝兩個相同方塊（相距 (off_y,off_x) 格）、B＝單一方塊。
    相關面必在兩處出現等高峰——次峰偵測必須抓到第二處。"""
    a = np.zeros((canvas, canvas), dtype=bool)
    b = np.zeros((canvas, canvas), dtype=bool)
    a[10:10 + size, 10:10 + size] = True
    a[10 + off_y:10 + off_y + size, 10 + off_x:10 + off_x + size] = True
    b[10:10 + size, 10:10 + size] = True
    return a, b


def test_best_shift_second_peak_at_exact_boundary():
    # 峰距「剛好」= sep（棋盤 10 格＝歐氏 10 格）。R5-F1：初版「嚴格 >」會漏掉。
    a, b = _twin_block_images(0, 10)
    _, _, hit, hit2 = mm._best_shift(a, b, sep_cells=10.0)
    assert hit2 == pytest.approx(hit, rel=0.05), \
        f"邊界峰應被視為獨立次峰，hit={hit} hit2={hit2}"


def test_best_shift_second_peak_on_diagonal():
    # 對角 (8,8) 格＝歐氏 11.3 格 > sep=10，但棋盤距離只有 8。
    # R5-F1：初版棋盤條件會把它算進「鄰域」而漏掉。
    a, b = _twin_block_images(8, 8)
    _, _, hit, hit2 = mm._best_shift(a, b, sep_cells=10.0)
    assert hit2 == pytest.approx(hit, rel=0.05), \
        f"對角峰應被視為獨立次峰，hit={hit} hit2={hit2}"


def test_best_shift_near_peak_not_counted_as_second():
    # 對照組：峰距 (3,3)＝歐氏 4.2 格 < sep=10 → 屬最佳峰鄰域，不得當獨立次峰
    a, b = _twin_block_images(3, 3)
    _, _, hit, hit2 = mm._best_shift(a, b, sep_cells=10.0)
    assert hit2 < hit * 0.8, f"鄰近峰不應算次峰，hit={hit} hit2={hit2}"


def test_unique_geometry_not_shift_ambiguous():
    # 對照組：L 形唯一幾何，自我對齊不應被平移歧義誤殺
    res = 0.05
    a = l_shape_walls(res)
    out = mm.align(a, a.copy(), res, coarse_deg=2.0)
    assert out is not None
    assert out["ambiguity"] < mm.DEFAULT_MAX_AMBIGUITY
    assert mm.accept(out, 0.0)[0]
