"""跨遍姿態圖約束的判定（零 ROS 依賴，照 follow_math／merge_math／explore_math 模式）。

RM2 驗收條件①（2026-08-29）。任務卡原文：

    「必須比對 posegraph 邊集合，證明**至少一條 constraint 連接第一遍節點與第二遍節點**
      （跨 session 約束），才算『照第一遍舊圖修正』真的發生。泛化的 loop closure log 不算。」

**為什麼 log 不算數**：slam_toolbox 印的迴圈閉合訊息不區分「跟自己這一遍的早期掃描閉環」
與「跟上一遍的舊節點閉環」。第二遍在原地繞一圈也會印一堆閉環訊息，但那完全沒用到第一遍
的圖——續建等於白做。唯一能分辨的證據是邊的兩個端點各屬哪一遍。

## slam_toolbox 實際提供什麼（2026-08-29 在 car1 實測，不是查文件）

`/slam_toolbox/graph_visualization`（MarkerArray）裡有兩種東西：

  * `ns='slam_toolbox'` 的 SPHERE：一個節點一個 marker，**marker.id ＝ 姿態圖節點 ID**，
    pose.position ＝ 節點位置。
  * `ns='slam_toolbox_edges'` 的 LINE_LIST：points 兩兩一組＝一條邊，
    **只有座標、沒有節點 ID**。

所以邊要先用座標對回節點（`resolve_edges`）。實測四組資料：端點對回節點的最差誤差
**0.000000 m**、最近與次近節點的間距最小 **0.0365 m** ⇒ 這個對應是明確的，不是硬湊。

## ⚠ 兩個「看起來會過、其實是錯的」陷阱（都是實測撞到才發現）

**陷阱一：光比對節點 ID 會假通過。** 獨立重建的第二遍，節點 ID 一樣從 0 開始編。
拿第一遍的 ID 集合去比，會發現「一端 ID≤7、一端 ID>7」的邊，看起來就是跨遍約束。
RM1 的對照組（`rm1_pass2_standalone`，完全沒載入舊圖）實測就這樣冒出 **5 條假跨遍邊**。
分辨方法＝比**同一個 ID 的節點位置**：真續建的舊節點原封不動（實測位置差 0.0000 m），
獨立重建的同 ID 節點是完全不同的節點（實測差 0.754 m 平均、1.227 m 最大）。

**陷阱二：lifelong 會裁掉舊節點，所以「第一遍節點必須全在」是錯的判準。**
實測載入 8 個節點的第一遍圖續建後，只剩 3 個（ID 3–7 被裁掉），這是 lifelong 的正常行為
（它會裁剪並重新優化姿態圖，正是 RM1 選它的原因）。要求全在會把合格的續建誤判成 INVALID。

## ⚠ baseline 一定要在第二遍開跑「之前」取

baseline 混進第二遍的節點會造成**假通過**：若節點 X 其實是第二遍產生的卻被記進 baseline，
X 與另一個第二遍節點之間的邊就會被判成跨遍，但它只是第二遍自己跟自己閉環。
反過來漏記第一遍節點只會造成假失敗，是安全的。**正確取法＝第一遍存姿態圖的當下。**

## 本判定證明什麼、不證明什麼

證明「第二遍確實接上了第一遍的圖」。**不證明接得好**——實測起始位姿給錯（RM1 那組
41.8% 吻合率的爛結果）同樣有跨遍邊。精度好壞由 RM 的捲尺真值等指標負責，不是這支。

座標／ID 慣例：節點 ID 是 slam_toolbox 給的整數；邊無方向性，(a, b) 與 (b, a) 視為同一條。
"""
import math

# --- 判決值 ---
PASS = "PASS"          # 找到至少一條跨遍約束
FAIL = "FAIL"          # 資料合法，但證據不成立（沒有跨遍約束／第二遍沒長出節點）
INVALID = "INVALID"    # 資料本身不可信，不能拿來下任何結論（fail-closed）

# --- 門檻（全部由 2026-08-29 car1 四組實資料校準，不是拍腦袋；改動要附新的實測）---
MATCH_TOL_M = 0.01
"""邊端點要多接近才算對上某個節點。實測最差 0.000000 m，取 1 cm＝純餘裕。"""

AMBIGUITY_MARGIN_M = 0.01
"""最近節點要比次近節點近這麼多才算明確。實測最小間距 0.0365 m，取 1 cm＝3.6 倍餘裕。"""

BASELINE_POSE_TOL_M = 0.10
"""同 ID 節點位置差多少之內算「同一個節點」。

實測分離度極大：真續建 0.0000 m、獨立重建 0.754 m（平均）／1.227 m（最大）。
取 0.10 m ＝ 比合格組寬鬆（容得下 lifelong 重新優化造成的公分級位移，地圖解析度 0.05 m
的兩格），又比不合格組嚴格 7 倍以上。
"""

MIN_SURVIVING_NODES = 2
"""至少要有這麼多第一遍節點留在最終圖裡，位置比對才有鑑別力。

只剩 1 個時，那個節點通常就是原點 (0,0)——而**每一張圖的節點 0 都在原點**，
拿它當「位置吻合」的證據等於沒有證據。實測 lifelong 續建留下 3 個。
"""

ORIGIN_EVIDENCE_RADIUS_M = 0.10
"""距離原點這麼近的節點不計入「有鑑別力的存活節點」，理由同上。"""

MAX_LISTED = 20
"""回報時最多列幾條跨遍邊（判定依據是計數，不是這個清單）。"""


def normalise_edge(a, b):
    """把一條邊正規化成 (小, 大)，讓無方向的邊可以進 set 去重。

    端點可以是單一節點 ID，也可以是「候選 ID 的 tuple」（座標分不出是哪個節點時）。
    """
    a = tuple(sorted(int(i) for i in a)) if isinstance(a, (tuple, list, set)) else (int(a),)
    b = tuple(sorted(int(i) for i in b)) if isinstance(b, (tuple, list, set)) else (int(b),)
    return (a, b) if a <= b else (b, a)


def dedupe_edges(edges):
    """去掉重複與自環。回傳排序後的 list，每個端點都是候選 tuple。

    自環（兩端候選完全相同）對跨遍判定零資訊（必定同一遍），直接丟掉而不是報錯。
    """
    out = {normalise_edge(a, b) for a, b in edges}
    return sorted(e for e in out if e[0] != e[1])


def _candidates(nodes, x, y, match_tol, ambiguity_margin):
    """回傳這個端點所有「可能是它」的節點 ID。

    正常情況只有一個（實測誤差 0.000000 m）。但**車回到原地重新開跑**時，第二遍的第一個
    節點會長在第一遍某個節點旁邊幾毫米處（2026-08-30 實測 6 mm），此時光看座標無法斷定
    邊接的是哪一個。這種情況回傳兩個候選，交由呼叫端決定還能不能下結論——
    **絕不可以把容忍值放寬然後隨便挑一個**，那是靜默的錯誤分類。
    """
    ds = sorted((math.hypot(px - x, py - y), i) for i, (px, py) in nodes.items())
    if not ds or ds[0][0] > match_tol:
        return (), (ds[0][0] if ds else float("inf"))
    limit = ds[0][0] + ambiguity_margin
    return tuple(sorted(i for d, i in ds if d <= limit)), ds[0][0]


def resolve_edges(nodes, edge_points,
                  match_tol=MATCH_TOL_M, ambiguity_margin=AMBIGUITY_MARGIN_M):
    """把 LINE_LIST 的線段端點座標對回節點。

    參數
      nodes        {節點 ID: (x, y)}
      edge_points  [(x1, y1, x2, y2), ...]

    回傳 (edges, problems)：
      edges     [((候選 ID, ...), (候選 ID, ...)), ...]——單一候選＝明確，多個＝座標分不出來
      problems  [說明字串, ...]（端點根本對不上任何節點＝圖抓得不完整，呼叫端應視為 INVALID）
    """
    edges, problems = [], []
    if not nodes:
        return [], ["節點集合是空的，無法把邊對回節點"]
    for x1, y1, x2, y2 in edge_points:
        pair, bad = [], False
        for (x, y) in ((x1, y1), (x2, y2)):
            cands, d = _candidates(nodes, x, y, match_tol, ambiguity_margin)
            if not cands:
                problems.append("端點 (%.3f, %.3f) 對不上任何節點（最近 %.4f m > %.4f m）"
                                % (x, y, d, match_tol))
                bad = True
                break
            pair.append(cands)
        if not bad:
            edges.append(tuple(pair))
    return edges, problems


def analyse(baseline_nodes, final_nodes, final_edges,
            pose_tol=BASELINE_POSE_TOL_M):
    """判定第二遍有沒有真的接上第一遍的圖。

    參數
      baseline_nodes  第一遍結束時的 {節點 ID: (x, y)}
      final_nodes     第二遍結束時的 {節點 ID: (x, y)}
      final_edges     第二遍結束時的邊 [(id, id), ...]（已用 resolve_edges 對好）

    回傳 dict：verdict／reason／n_pass1／n_pass2／n_edges／counts／cross_edges／checks
    """
    base = {int(k): (float(v[0]), float(v[1])) for k, v in dict(baseline_nodes).items()}
    final = {int(k): (float(v[0]), float(v[1])) for k, v in dict(final_nodes).items()}
    edges = dedupe_edges(final_edges)

    surviving = sorted(set(base) & set(final))
    new_ids = sorted(set(final) - set(base))

    def result(verdict, reason, counts=None, cross=(), checks=None):
        return {
            "verdict": verdict,
            "reason": reason,
            "n_pass1": len(surviving),
            "n_pass2": len(new_ids),
            "n_edges": len(edges),
            "counts": counts or {"pass1": 0, "pass2": 0, "cross": 0, "undecidable": 0},
            "cross_edges": list(cross)[:MAX_LISTED],
            "checks": checks or {},
        }

    # --- INVALID：資料本身不可信，一律 fail-closed ---
    if not base:
        return result(INVALID, "baseline 是空的：沒有第一遍的節點可比對")
    if not final:
        return result(INVALID, "第二遍的圖是空的：沒有節點")

    dangling = {n for e in edges for side in e for n in side} - set(final)
    if dangling:
        return result(INVALID, "有 %d 個邊端點不在節點集合裡（圖抓到一半）：%s"
                      % (len(dangling), sorted(dangling)[:10]))

    # 陷阱一：同 ID 節點位置對不上 ⇒ 不是同一張圖（多半根本沒載入舊圖）
    moved = []
    for i in surviving:
        d = math.hypot(base[i][0] - final[i][0], base[i][1] - final[i][1])
        if d > pose_tol:
            moved.append((i, d))
    if moved:
        worst = max(moved, key=lambda t: t[1])
        return result(
            INVALID,
            "有 %d 個第一遍節點的 ID 還在、位置卻對不上（最差 ID %d 差 %.3f m > %.3f m）"
            "＝這不是同一張圖，第二遍很可能根本沒載入第一遍的姿態圖"
            % (len(moved), worst[0], worst[1], pose_tol),
            checks={"moved_nodes": moved[:10]},
        )

    # 陷阱二的另一面：位置比對要有鑑別力，原點節點不算數
    informative = [i for i in surviving
                   if math.hypot(*base[i]) > ORIGIN_EVIDENCE_RADIUS_M]
    if len(surviving) < MIN_SURVIVING_NODES or not informative:
        return result(
            INVALID,
            "只剩 %d 個第一遍節點（其中有鑑別力的 %d 個）：不足以證明載到的是同一張圖"
            "——每張圖的節點 0 都在原點，光靠它不算證據"
            % (len(surviving), len(informative)),
        )

    # --- 分類 ---
    # 端點可能有多個候選（座標分不出是哪個節點）。只有當「所有候選都屬同一遍」時，
    # 這一端的歸屬才確定；否則整條邊列為 undecidable，**永遠不算證據**。
    # 這樣模糊只會減少證據、不會製造證據，也不會讓整份分析作廢。
    def side_class(cands):
        olds = [c in base for c in cands]
        if all(olds):
            return "p1"
        if not any(olds):
            return "p2"
        return "?"

    counts = {"pass1": 0, "pass2": 0, "cross": 0, "undecidable": 0}
    cross = []
    for a, b in edges:
        ca, cb = side_class(a), side_class(b)
        if "?" in (ca, cb):
            counts["undecidable"] += 1
        elif ca == "p1" and cb == "p1":
            counts["pass1"] += 1
        elif ca == "p2" and cb == "p2":
            counts["pass2"] += 1
        else:
            counts["cross"] += 1
            cross.append((a, b))

    checks = {"surviving": surviving, "informative_surviving": informative,
              "pruned": sorted(set(base) - set(final))}

    if not new_ids:
        return result(FAIL, "第二遍沒有產生任何新節點（車沒走／掃描沒進來），這趟不構成續建，需重跑",
                      counts, cross, checks)

    if counts["cross"] == 0:
        return result(FAIL,
                      "第二遍長出 %d 個節點，但沒有任何一條邊連回第一遍＝兩遍各走各的，續建沒有生效"
                      % len(new_ids), counts, cross, checks)

    return result(PASS,
                  "找到 %d 條跨遍約束（第一遍存活 %d 節點／第二遍新增 %d）"
                  "＝第二遍確實接上了第一遍的圖" % (counts["cross"], len(surviving), len(new_ids)),
                  counts, cross, checks)


def format_report(res):
    """把 analyse 的結果印成人看得懂的幾行（工具與測試共用）。"""
    lines = [
        "判決：%s" % res["verdict"],
        "理由：%s" % res["reason"],
        "節點：第一遍存活 %d／第二遍新增 %d" % (res["n_pass1"], res["n_pass2"]),
        "邊　：合計 %d（第一遍內 %d／第二遍內 %d／跨遍 %d／無法判定 %d）"
        % (res["n_edges"], res["counts"]["pass1"], res["counts"]["pass2"],
           res["counts"]["cross"], res["counts"].get("undecidable", 0)),
    ]
    if res["counts"].get("undecidable"):
        lines.append("（無法判定＝兩個節點靠太近、座標分不出邊接哪一個；一律不採計為證據）")
    pruned = res.get("checks", {}).get("pruned")
    if pruned:
        lines.append("（第一遍有 %d 個節點被 lifelong 裁掉，屬正常：%s）" % (len(pruned), pruned[:10]))
    if res["cross_edges"]:
        shown = ", ".join("%s-%s" % ("|".join(map(str, a)), "|".join(map(str, b)))
                          for a, b in res["cross_edges"])
        more = "" if res["counts"]["cross"] <= MAX_LISTED else " …（僅列前 %d 條）" % MAX_LISTED
        lines.append("跨遍邊：%s%s" % (shown, more))
    return "\n".join(lines)
