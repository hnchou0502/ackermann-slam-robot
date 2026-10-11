"""雙車合圖的純數學（零 ROS 依賴，照 follow_math 模式；Windows 直接 pytest）。

抽出自 map_merge_live（2026-07-21，Codex F3/F4）：對齊搜尋、TF 慣例、
以及三道「壞對齊」防護——歧義峰、評分飽和、空曠衝突。

TF 慣例（與 merge_maps_offline.py 一致）：
  B 系點 (xb, yb) → A 系 = (tx + c·xb − s·yb,  ty + s·xb + c·yb)

防護的由來（實測教訓）：
  - 飽和（CM1 2026-07-20）：評分容忍放太寬時，相距很遠的角度同時拿滿分＝假尖峰。
  - 歧義（Codex F3）：直牆/重複走廊等幾何，多個姿態可得相近高分——
    單看最高分會選到錯解，必須比對「角度相距夠遠的第二名」。
  - 空曠衝突（Codex F3）：正確對齊下 B 的牆不該大量落在 A 明確看過是空地的格子。
"""
import math

import numpy as np

MARGIN = 40             # 對齊畫布留白（格），防 FFT 環繞污染
SAT_SCORE = 0.98        # 分數 ≥ 此值視為「近滿分」
SAT_MAX_ANGLES = 5      # 近滿分角度超過此數＝評分飽和（假尖峰）
AMBIG_SEP_DEG = 15.0    # 角度第二峰須與最佳角相距至少此角度才算獨立峰
SHIFT_SEP_M = 0.5       # 平移第二峰須與最佳平移相距至少此距離（Codex R4-F1：
                        # 兩個相同房間/重複走廊＝同角度、不同平移的等高峰）

# 發布/採信門檻的唯一真相源（Codex R4-F2：政策常數集中一處，工具與節點都吃這裡；
# 數值＝2026-07-21 實資料校準：正解歧義 0.33/0.53 vs 真歧義 0.896→取 0.80；
# 正解衝突 0.11/0.198 vs 錯解 0.33/0.425→取 0.25。test_merge_math 鎖住這三個常數。）
DEFAULT_MIN_SCORE = 0.30
DEFAULT_MAX_AMBIGUITY = 0.80
DEFAULT_MAX_CONFLICT = 0.25


def transform_pts(pts, tx, ty, yaw):
    """B 系點集 → A 系（TF 慣例的唯一實作，測試鎖這裡）。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([tx + c * pts[:, 0] - s * pts[:, 1],
                     ty + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def walls_from_grid(data, width, height, res, ox, oy, occ_thresh=65):
    """OccupancyGrid.data（row 0 在 origin 側，**無** PGM 的 y 翻轉）→ 牆格世界座標 (N,2)。"""
    arr = np.asarray(data, dtype=np.int16).reshape(height, width)
    rows, cols = np.nonzero(arr >= occ_thresh)
    x = ox + (cols + 0.5) * res
    y = oy + (rows + 0.5) * res
    return np.stack([x, y], axis=1)


def free_from_grid(data, width, height, res, ox, oy, free_max=25):
    """同上，但取「明確空曠」格（0..free_max）。"""
    arr = np.asarray(data, dtype=np.int16).reshape(height, width)
    rows, cols = np.nonzero((arr >= 0) & (arr <= free_max))
    x = ox + (cols + 0.5) * res
    y = oy + (rows + 0.5) * res
    return np.stack([x, y], axis=1)


def rasterize(pts, x0, y0, res, shape):
    H, W = shape
    ix = np.floor((pts[:, 0] - x0) / res).astype(np.int64)
    iy = np.floor((pts[:, 1] - y0) / res).astype(np.int64)
    ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    img = np.zeros(shape, dtype=bool)
    img[iy[ok], ix[ok]] = True
    return img


def _best_shift(a_img, b_img, sep_cells):
    """FFT 互相關求最佳平移＋「相距 ≥ sep_cells（歐氏、含邊界）的次高平移峰」。

    次高峰是 R4-F1 的解毒劑：兩個相同房間/重複走廊會在同一角度產生多個等高
    平移峰——只取 argmax 會把歧義藏起來。回傳 (dx, dy, hit, hit2)。

    ⚠ 距離語義（Codex R5-F1）：必須是**環狀歐氏距離、>=**。初版用「棋盤距離、
    嚴格 >」——剛好 sep 的峰與對角 sep·1.13 的峰都會漏抓（test_merge_math 有
    邊界/對角負例鎖住這裡）。sep_cells 可為浮點，呼叫端傳 SHIFT_SEP_M/res 不取整。
    """
    F = np.fft.rfft2(a_img.astype(np.float32))
    G = np.fft.rfft2(b_img.astype(np.float32))
    corr = np.fft.irfft2(F * np.conj(G), s=a_img.shape)
    H, W = corr.shape
    idx = int(np.argmax(corr))
    py, px = divmod(idx, W)
    hit = float(corr[py, px])
    # 環狀歐氏距離 ≥ sep 的區域取最大值＝次高平移峰
    yy = np.abs(np.arange(H)[:, None] - py)
    xx = np.abs(np.arange(W)[None, :] - px)
    cyy = np.minimum(yy, H - yy).astype(np.float64)
    cxx = np.minimum(xx, W - xx).astype(np.float64)
    far = (cyy * cyy + cxx * cxx) >= float(sep_cells) ** 2
    hit2 = float(corr[far].max()) if far.any() else 0.0
    dy, dx = py, px
    if dy > H // 2:
        dy -= H
    if dx > W // 2:
        dx -= W
    return dx, dy, hit, hit2


def _canvas(a_pts, b_pts, res):
    rb = float(np.max(np.linalg.norm(b_pts, axis=1))) if len(b_pts) else 0.0
    x0 = min(float(a_pts[:, 0].min()), -rb) - MARGIN * res
    y0 = min(float(a_pts[:, 1].min()), -rb) - MARGIN * res
    x1 = max(float(a_pts[:, 0].max()), rb) + MARGIN * res
    y1 = max(float(a_pts[:, 1].max()), rb) + MARGIN * res
    shape = (int(math.ceil((y1 - y0) / res)), int(math.ceil((x1 - x0) / res)))
    return x0, y0, shape


def align(a_pts, b_pts, res, coarse_deg=2.0):
    """暴力搜尋最佳剛體變換（tol=0——CM1 教訓：膨脹會假飽和）。

    回傳 dict：
      score / yaw / tx / ty          最佳解
      runner_score / runner_yaw      次高歧義峰（兩種來源取大者，見 runner_kind）
      runner_kind                    'angle'＝不同角度的他峰；'shift'＝同角度的平移他峰
                                     （後者＝Codex R4-F1：相同房間/重複走廊的退化）
      ambiguity                      runner_score / score（越接近 1 越歧義）
      saturated                      近滿分角度是否超量（假尖峰徵兆）
    """
    x0, y0, shape = _canvas(a_pts, b_pts, res)
    a_img = rasterize(a_pts, x0, y0, res, shape)
    sep_cells = max(1.0, SHIFT_SEP_M / res)   # 浮點、不取整（R5-F1：取整會挪動邊界）

    per_angle = []   # (score, yaw, tx, ty, score2_shift)
    def sweep(angles):
        for th in angles:
            c, s = math.cos(th), math.sin(th)
            rot = np.stack([c * b_pts[:, 0] - s * b_pts[:, 1],
                            s * b_pts[:, 0] + c * b_pts[:, 1]], axis=1)
            b_img = rasterize(rot, x0, y0, res, shape)
            nb = int(b_img.sum())
            if nb == 0:
                continue
            dx, dy, hit, hit2 = _best_shift(a_img, b_img, sep_cells)
            per_angle.append((hit / nb, th, dx * res, dy * res, hit2 / nb))

    sweep([math.radians(d) for d in np.arange(0.0, 360.0, coarse_deg)])
    if not per_angle:
        return None
    coarse = sorted(per_angle, reverse=True)
    best_c = coarse[0]
    sweep([best_c[1] + math.radians(d)
           for d in np.arange(-coarse_deg, coarse_deg + 1e-9, 0.2)])
    allres = sorted(per_angle, reverse=True)
    score, yaw, tx, ty, shift2 = allres[0]

    # 歧義峰來源一：角度相距 ≥ AMBIG_SEP_DEG 的最高他峰（環狀角距）
    ang_score, ang_yaw = 0.0, None
    for sc, th, _, _, _ in allres[1:]:
        d = abs((math.degrees(th - yaw) + 180.0) % 360.0 - 180.0)
        if d >= AMBIG_SEP_DEG:
            ang_score, ang_yaw = sc, th
            break
    # 歧義峰來源二：最佳角度自己的次高平移峰（R4-F1）
    if shift2 > ang_score:
        runner_score, runner_yaw, runner_kind = shift2, yaw, "shift"
    else:
        runner_score, runner_yaw, runner_kind = ang_score, ang_yaw, "angle"
    # 飽和：粗掃裡近滿分的「獨立角」個數
    sat_n = sum(1 for sc, *_ in coarse if sc >= SAT_SCORE)

    return {
        "score": score, "yaw": yaw, "tx": tx, "ty": ty,
        "runner_score": runner_score, "runner_yaw": runner_yaw,
        "runner_kind": runner_kind,
        "ambiguity": (runner_score / score) if score > 0 else 1.0,
        "saturated": sat_n > SAT_MAX_ANGLES,
    }


def accept(result, conflict,
           min_score=DEFAULT_MIN_SCORE,
           max_ambiguity=DEFAULT_MAX_AMBIGUITY,
           max_conflict=DEFAULT_MAX_CONFLICT):
    """發布/採信決策的唯一實作（Codex R4-F2：政策必須是可測的純函式）。

    回傳 (ok: bool, reason: str)。reason 供 log 與測試斷言。
    """
    if result is None:
        return False, "no_solution"
    if result["saturated"]:
        return False, "saturated"
    if result["score"] < min_score:
        return False, "low_score"
    if result["ambiguity"] > max_ambiguity:
        return False, f"ambiguous_{result.get('runner_kind', 'unknown')}"
    if conflict > max_conflict:
        return False, "free_conflict"
    return True, "ok"


def free_conflict_ratio(a_free_pts, b_pts, tx, ty, yaw, res):
    """B 牆經變換後，落在「A 明確空曠格」上的比例。正確對齊應該很低。"""
    if len(a_free_pts) == 0 or len(b_pts) == 0:
        return 0.0
    x0, y0, shape = _canvas(a_free_pts, b_pts, res)
    free_img = rasterize(a_free_pts, x0, y0, res, shape)
    tb = transform_pts(b_pts, tx, ty, yaw)
    H, W = shape
    ix = np.floor((tb[:, 0] - x0) / res).astype(np.int64)
    iy = np.floor((tb[:, 1] - y0) / res).astype(np.int64)
    ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    if not ok.any():
        return 1.0
    hits = free_img[iy[ok], ix[ok]]
    return float(hits.sum()) / float(len(b_pts))


def fuse_grids(a, b, tx, ty, yaw, occ_thresh=65, free_max=25):
    """融合兩張格網。a/b＝dict(data,width,height,res,ox,oy)；回傳 (out_int8_HxW, minx, miny)。

    規則與 merge_maps_offline 相同：牆優先 > 空曠 > 未知(-1)。
    """
    c, s = math.cos(yaw), math.sin(yaw)
    res = a["res"]
    ha, wa, hb, wb = a["height"], a["width"], b["height"], b["width"]
    arr_a = np.asarray(a["data"], dtype=np.int16).reshape(ha, wa)
    arr_b = np.asarray(b["data"], dtype=np.int16).reshape(hb, wb)
    oax, oay, obx, oby = a["ox"], a["oy"], b["ox"], b["oy"]

    ca = [(oax, oay), (oax + wa * res, oay), (oax, oay + ha * res),
          (oax + wa * res, oay + ha * res)]
    cb = []
    for xb, yb in [(obx, oby), (obx + wb * res, oby), (obx, oby + hb * res),
                   (obx + wb * res, oby + hb * res)]:
        cb.append((tx + c * xb - s * yb, ty + s * xb + c * yb))
    xs = [p[0] for p in ca + cb]
    ys = [p[1] for p in ca + cb]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    W = int(math.ceil((maxx - minx) / res))
    H = int(math.ceil((maxy - miny) / res))

    jj, ii = np.meshgrid(np.arange(W), np.arange(H))
    gx = minx + (jj + 0.5) * res
    gy = miny + (ii + 0.5) * res
    au = np.floor((gx - oax) / res).astype(np.int64)
    av = np.floor((gy - oay) / res).astype(np.int64)
    va = np.full((H, W), -1, dtype=np.int16)
    m = (au >= 0) & (au < wa) & (av >= 0) & (av < ha)
    va[m] = arr_a[av[m], au[m]]

    dxx, dyy = gx - tx, gy - ty
    xb = c * dxx + s * dyy
    yb = -s * dxx + c * dyy
    bu = np.floor((xb - obx) / res).astype(np.int64)
    bv = np.floor((yb - oby) / res).astype(np.int64)
    vb = np.full((H, W), -1, dtype=np.int16)
    m = (bu >= 0) & (bu < wb) & (bv >= 0) & (bv < hb)
    vb[m] = arr_b[bv[m], bu[m]]

    out = np.full((H, W), -1, dtype=np.int8)
    out[((va >= 0) & (va <= free_max)) | ((vb >= 0) & (vb <= free_max))] = 0
    out[(va >= occ_thresh) | (vb >= occ_thresh)] = 100
    return out, minx, miny
