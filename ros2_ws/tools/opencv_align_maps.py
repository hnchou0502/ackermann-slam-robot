#!/usr/bin/env python3
"""OpenCV 特徵比對版地圖對齊——與 auto_align_maps.py（暴力搜尋）解同一個問題的第二種方法。

**定位**：CM3 規劃的「OpenCV 地圖特徵比對自動求變換」的實作。與暴力搜尋法跑同一組
資料、用同一把尺（merge_maps_offline.py 的 5cm 吻合率）評分，供論文做方法比較。

**方法**：
  1. 佔據圖 → 牆遮罩 → 放大 UPSCALE 倍＋膨脹＋高斯模糊
     （⚠ 這步是成敗關鍵：特徵偵測器為照片設計，39×43 px 的原圖直接餵必死——
      牆線太細、無紋理，偵測不到有支撐的關鍵點）
  2. SIFT 偵測關鍵點＋描述子（⚠ OpenCV 5.0 已無 AKAZE；SIFT 尺度不變、浮點描述子，
     對線條圖較穩；ORB 為備援）
  3. BFMatcher + Lowe ratio test 配對
  4. estimateAffinePartial2D + RANSAC 求 B像素系→A像素系 的相似變換（旋轉+平移+等比縮放）
  5. 合成像素↔世界座標的 y 翻轉仿射，換算成與 merge_maps_offline 同慣例的世界系 TF：
       B 系點 (xb,yb) → A 系 = (tx + c·xb − s·yb, ty + s·xb + c·yb)

**誠實條款**：
  - 相似變換含縮放自由度；兩圖解析度相同，理論 scale≈1。輸出會印 scale，
    偏離 1 超過 2% 即警告（代表特徵配對品質差，或兩車里程尺度真的有差）。
  - 評分一律只取 (tx,ty,yaw) 餵 merge_maps_offline（它是剛體模型）——scale 偏差會
    直接反映成低分，這對兩法比較是公平的。

用法：python3 opencv_align_maps.py <A前綴> <B前綴> [--upscale N] [--json 輸出]
"""
import sys
import re
import math
import json
import numpy as np
import cv2

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

OCC_THRESH = 100
UPSCALE_DEFAULT = 8       # 放大倍率：房間圖 39px → 312px 才有得偵測；大圖會自動降
MAX_SIDE = 1600           # 放大後最長邊上限（防大圖爆記憶體）
RATIO = 0.75              # Lowe ratio test 閾值


def read_yaml(path):
    d = {}
    for line in open(path, encoding="utf-8"):
        m = re.match(r"^(\w+):\s*(.+)$", line.strip())
        if m:
            d[m.group(1)] = m.group(2)
    res = float(d["resolution"])
    origin = [float(x) for x in d["origin"].strip("[]").split(",")[:2]]
    return res, origin


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    i += 1
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)


def prep(arr, upscale):
    """牆遮罩 → 放大 → 膨脹 → 模糊，回傳 (影像, 實際採用的倍率)。"""
    h, w = arr.shape
    k = min(upscale, max(1, MAX_SIDE // max(h, w)))
    mask = (arr < OCC_THRESH).astype(np.uint8) * 255
    big = cv2.resize(mask, (w * k, h * k), interpolation=cv2.INTER_NEAREST)
    big = cv2.dilate(big, np.ones((3, 3), np.uint8), iterations=2)
    big = cv2.GaussianBlur(big, (5, 5), 0)
    return big, k


def detect_and_match(img_a, img_b, order=("SIFT", "ORB")):
    """SIFT 優先（⚠ OpenCV 5.0 已移除 AKAZE，2026-07-21 實查）、ORB 備援。
    回傳 (ptsA, ptsB, 偵測器名, 原始配對數)。"""
    for name in order:
        if name == "SIFT":
            det = cv2.SIFT_create()
            norm = cv2.NORM_L2
        else:
            det = cv2.ORB_create(nfeatures=4000)
            norm = cv2.NORM_HAMMING
        ka, da = det.detectAndCompute(img_a, None)
        kb, db = det.detectAndCompute(img_b, None)
        if da is None or db is None or len(ka) < 10 or len(kb) < 10:
            print(f"  {name}: 關鍵點不足（A={len(ka) if ka else 0}, B={len(kb) if kb else 0}），換下一個")
            continue
        matcher = cv2.BFMatcher(norm)
        raw = matcher.knnMatch(db, da, k=2)   # query=B, train=A
        good = [m for m, n in (p for p in raw if len(p) == 2) if m.distance < RATIO * n.distance]
        print(f"  {name}: 關鍵點 A={len(ka)} B={len(kb)}，ratio test 後配對 {len(good)}")
        if len(good) >= 8:
            pts_b = np.float32([kb[m.queryIdx].pt for m in good])
            pts_a = np.float32([ka[m.trainIdx].pt for m in good])
            return pts_a, pts_b, name, len(good)
    return None, None, None, 0


def px_to_world_mat(origin, res, h, k):
    """放大 k 倍後的像素座標 (u,v) → 世界座標 的 2x3 仿射（含 y 翻轉）。

    原圖格 (col,row)：x = ox + (col+0.5)res, y = oy + (h-1-row+0.5)res
    放大後 u = col*k + k/2 處為格中心 → col = u/k - 0.5 …直接用連續版：
      x = ox + (u/k) * res      （u 以像素左上為 0，格中心自動落對）
      y = oy + (h - v/k) * res
    """
    s = res / k
    return np.array([[s, 0.0, origin[0]],
                     [0.0, -s, origin[1] + h * res]], dtype=np.float64)


def compose(Wa, P, Wb):
    """T = Wa ∘ P ∘ Wb⁻¹（皆 2x3 仿射）。"""
    def to3(M):
        return np.vstack([M, [0.0, 0.0, 1.0]])
    T = to3(Wa) @ to3(P) @ np.linalg.inv(to3(Wb))
    return T[:2, :]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) < 2:
        print(__doc__)
        return 1
    a_prefix, b_prefix = args[0], args[1]
    upscale = int(sys.argv[sys.argv.index("--upscale") + 1]) if "--upscale" in sys.argv else UPSCALE_DEFAULT
    json_out = sys.argv[sys.argv.index("--json") + 1] if "--json" in sys.argv else None
    det_order = ("SIFT", "ORB")
    if "--detector" in sys.argv:
        det_order = (sys.argv[sys.argv.index("--detector") + 1],)

    res_a, org_a = read_yaml(a_prefix + ".yaml")
    res_b, org_b = read_yaml(b_prefix + ".yaml")
    if abs(res_a - res_b) > 1e-9:
        print(f"解析度不同（{res_a} vs {res_b}），不處理")
        return 1
    arr_a = read_pgm(a_prefix + ".pgm")
    arr_b = read_pgm(b_prefix + ".pgm")

    img_a, ka = prep(arr_a, upscale)
    img_b, kb = prep(arr_b, upscale)
    if ka != kb:
        # 兩圖尺寸差很多時倍率可能不同；統一取小的重做，維持同尺度
        k = min(ka, kb)
        img_a, ka = prep(arr_a, k)
        img_b, kb = prep(arr_b, k)
    print(f"A {arr_a.shape[1]}x{arr_a.shape[0]} → x{ka}；B {arr_b.shape[1]}x{arr_b.shape[0]} → x{kb}")

    pts_a, pts_b, det_name, n_good = detect_and_match(img_a, img_b, det_order)
    if pts_a is None:
        print("\n=== 失敗：特徵配對不足（<8 對）——特徵法在此資料上不成立 ===")
        if json_out:
            json.dump({"ok": False, "reason": "insufficient_matches"},
                      open(json_out, "w", encoding="utf-8"))
        return 2

    P, inliers = cv2.estimateAffinePartial2D(
        pts_b, pts_a, method=cv2.RANSAC,
        ransacReprojThreshold=3.0, maxIters=5000, confidence=0.999)
    if P is None:
        print("\n=== 失敗：RANSAC 無解 ===")
        if json_out:
            json.dump({"ok": False, "reason": "ransac_failed"},
                      open(json_out, "w", encoding="utf-8"))
        return 2
    n_in = int(inliers.sum())
    print(f"RANSAC: inliers {n_in}/{len(pts_b)} = {100*n_in/len(pts_b):.0f}%")

    Wa = px_to_world_mat(org_a, res_a, arr_a.shape[0], ka)
    Wb = px_to_world_mat(org_b, res_b, arr_b.shape[0], kb)
    T = compose(Wa, P, Wb)

    scale = math.hypot(T[0, 0], T[1, 0])
    yaw = math.atan2(T[1, 0], T[0, 0])
    tx, ty = float(T[0, 2]), float(T[1, 2])

    # 退化解防護（2026-07-21 客廳組實測踩到）：RANSAC 有時「收斂」到 scale≈0
    # （所有點縮成一點的假解，inlier 判定仍過門檻）。這種解必須明確報失敗，
    # 不能印出垃圾 TF 讓下游誤用。
    if not (0.5 <= scale <= 2.0):
        print(f"\n=== 失敗：退化解（scale={scale:.4f}，剛體應 ≈1）——"
              f"特徵配對品質不足以支撐幾何求解 ===")
        if json_out:
            json.dump({"ok": False, "reason": "degenerate_scale",
                       "detector": det_name, "scale": scale,
                       "matches": n_good, "inliers": n_in},
                      open(json_out, "w", encoding="utf-8"), indent=2)
        return 2

    print(f"\n=== OpenCV（{det_name}）解 ===")
    print(f"yaw   {math.degrees(yaw):+.2f}°  ({yaw:+.6f} rad)")
    print(f"平移  tx={tx:+.3f} m  ty={ty:+.3f} m")
    print(f"scale {scale:.4f}", end="")
    if abs(scale - 1.0) > 0.02:
        print("  ⚠ 偏離 1 超過 2%——配對品質差或兩車里程尺度有差；剛體評分會吃虧")
    else:
        print("  （≈1，符合同解析度剛體預期）")
    print(f"\nTF {tx:.4f} {ty:.4f} {yaw:.6f}")
    print(f"\n驗分：python3 merge_maps_offline.py {a_prefix} {b_prefix} <輸出> {tx:.4f} {ty:.4f} {yaw:.6f}")

    if json_out:
        json.dump({"ok": True, "detector": det_name, "matches": n_good,
                   "inliers": n_in, "scale": scale, "tx": tx, "ty": ty,
                   "yaw_rad": yaw, "yaw_deg": math.degrees(yaw)},
                  open(json_out, "w", encoding="utf-8"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
