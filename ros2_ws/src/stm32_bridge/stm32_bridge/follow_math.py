"""跟隨控制與模擬的純數學模組——零 ROS 依賴，可在任何機器 pytest。

把 follower / kinematic_sim / figure8_driver 的「會算錯」的部分全部集中在這裡，
ROS 節點只留訊息收發的膠水。這樣控制邏輯的驗證不需要 ROS、不需要車、不需要 Pi
（呼應 docs/design/v2v_architecture.md §6：V2 全程純筆電可開發）。

內容：
    normalize_angle        角度收斂到 [-pi, pi]
    yaw_from_quaternion    四元數 -> 平面航向角 yaw
    min_turn_radius        阿克曼最小轉彎半徑
    apply_ackermann        把 (v, w) 指令夾進阿克曼可行域
    figure8_params         由「速度＋最小轉彎半徑」反算 8 字的角速度振幅與週期
    figure8_omega          8 字在時刻 t 的角速度
    world_to_base          世界座標點 -> 車體座標
    align_leader_point     起點對齊：把 leader 位置正規化進 follower 座標系（吸收任意起點）
    adaptive_lookahead     純追蹤前視距離隨速縮放（低速不切彎、高速行為不變）
    pure_pursuit_curvature 純追蹤曲率
    BreadcrumbTrail        麵包屑軌跡（累積弧長、投影、lookahead 取點、沿路徑間距）
    rms / point_segment_distance / cross_track_series / score_following
                           V2/V3 驗收評分的純數學（抽出來才能不靠 ROS 做 pytest）
"""
import math


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """四元數 (x,y,z,w) 取平面航向角 yaw（繞 z 軸）。

    吃四個分量而非 ROS msg 物件，才能不依賴 ROS 做 pytest。follower / score_follow
    共用同一份（原本各有一份重複），呼叫端傳 orientation.x/.y/.z/.w。
    """
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def min_turn_radius(wheelbase: float, max_steer: float) -> float:
    """阿克曼最小轉彎半徑 R_min = L / tan(delta_max)。

    本車：L=0.235 m、delta_max=30deg -> 0.4070 m。指令要求比這更彎，車就是轉不過去。
    """
    return wheelbase / math.tan(max_steer)


def apply_ackermann(v: float, w: float, wheelbase: float,
                    max_steer: float, min_turn_speed: float = 0.01):
    """把 (v, w) 夾進阿克曼可行域，回傳實際做得到的 (v, w)。

    兩條約束（與 STM32 韌體 main.c 及 bridge_node.cmd_cb 同一套）：
      1. |v| < min_turn_speed -> w = 0（前輪轉向的車不能原地旋轉）
      2. 等效轉向角 delta = atan(L*w/v) 飽和在 ±max_steer，再反解 w
    """
    if abs(v) < min_turn_speed:
        return v, 0.0
    delta = math.atan(wheelbase * w / v)
    delta = max(-max_steer, min(max_steer, delta))
    return v, v * math.tan(delta) / wheelbase


def figure8_params(v: float, radius: float):
    """回傳 8 字軌跡的 (角速度振幅 A, 週期 T)，使最小瞬時轉彎半徑恰為 radius。

    軌跡定義：v 固定，w(t) = A*sin(2*pi*t/T)。
      積分得 theta(t) = (A/omega)*(1 - cos(omega*t))，其中 omega = 2*pi/T。
      要讓車頭「轉滿一圈再轉回來」（＝畫出一個左環＋一個右環＝8 字），
      需 theta 峰值 = 2*pi，即 2A/omega = 2*pi -> A/omega = pi -> T = 2*pi^2/A。
    瞬時轉彎半徑 = v/|w|，最小值出現在 |w|=A 處，即 radius = v/A -> A = v/radius。

    ⚠ 這條軌跡的兩個環是平滑相接的，但**有淨位移**（不是原地閉合的 lemniscate）：
       theta(t) 對稱於 t=T/2，故 x(T)=2*x(T/2)。視覺上仍是 8 字，只是會沿著場地前進。
       demo 影片要原地閉合的話得改用「兩個相切定曲率圓」，但那樣 w 在切換點不連續。
    """
    if radius <= 0.0 or v <= 0.0:
        raise ValueError('v 與 radius 都必須為正')
    a = v / radius
    t = 2.0 * math.pi ** 2 / a
    return a, t


def figure8_omega(t: float, amplitude: float, period: float) -> float:
    return amplitude * math.sin(2.0 * math.pi * t / period)


def world_to_base(px: float, py: float, rx: float, ry: float, rth: float):
    """把世界座標的點 (px,py) 轉進以 (rx,ry,rth) 為原點的車體座標。x 軸=車頭方向。"""
    dx = px - rx
    dy = py - ry
    c, s = math.cos(-rth), math.sin(-rth)
    return dx * c - dy * s, dx * s + dy * c


def align_leader_point(lx: float, ly: float, origin, gap0: float):
    """把 leader 在自身 odom 系的位置 (lx,ly) 對齊到 follower 座標系。

    起點對齊（D5 校正起點法）：以 leader **第一筆位姿** origin=(ox,oy,oth) 為基準，
    把整條 leader 軌跡轉進「起點在原點、起點航向為 +x」的正規系，再平移到 follower
    正前方 gap0——於是不論 leader 的 odom 起點在哪、朝哪，follower 看到的都是「我正前方
    gap0、與我同向出發的一條路」，與 V2 兩台 sim 從原點出發時完全等價。

    V2 相容：origin=(0,0,0) 時退化成單純 +gap0 平移零旋轉（＝這次改動前的行為）。
    ⚠ 這只吸收 **leader 端**的任意起點。follower 自身起點/航向是否對齊 leader 起點，
       實車仍要靠物理對齊治具（±1°）——這段數學管不到，與 V2 相同前提。
    """
    rx, ry = world_to_base(lx, ly, origin[0], origin[1], origin[2])
    return rx + gap0, ry


def adaptive_lookahead(v: float, gamma: float = 2.3,
                       lo: float = 0.35, hi: float = 0.8) -> float:
    """純追蹤前視距離隨速縮放：ld = γ·|v|，夾在 [lo, hi]。

    固定前視距離的問題（2026-07-11 V3 實車 bag 實測）：ld=0.8m 但實車只開 0.2 m/s，
    等於看 4 秒後的路開車，R=0.85m 的彎被切 0.182m（幾何預估 R(1-cos(ld/2R))≈0.09m
    ＋實資料雜訊）。隨速縮放後 0.2 m/s → ld≈0.46m，預估切彎量 ~0.03m。
    γ=2.3 的由來：V2 模擬速度 0.35 m/s × 2.3 ≈ 0.8 ＝原固定值 → V2 行為不變（已迴歸驗證）。
    下限 lo 防低速時前視過短造成轉向震盪；上限 hi 維持原高速行為。
    （設計文件 §4 D6 原文即預告「進階：ld=γ·v 隨速縮放＋上下限」，本函式為其實作。）
    """
    return max(lo, min(hi, gamma * abs(v)))


def pure_pursuit_curvature(bx: float, by: float) -> float:
    """純追蹤曲率 kappa = 2*by / ld^2（目標點已在車體座標）。角速度 w = v * kappa。

    by>0（目標在左）-> kappa>0 -> 左轉，與本車 wz>0 前輪左偏的實機驗證一致（T3, 2026-07-08）。
    目標點落在車體原點上時回 0（避免除以零）。
    """
    ld2 = bx * bx + by * by
    if ld2 < 1e-9:
        return 0.0
    return 2.0 * by / ld2


class BreadcrumbTrail:
    """leader 走過的軌跡點（麵包屑），供 follower 沿路徑跟隨。

    為什麼跟「軌跡」而不是直接追「leader 當下位置」：直接追會切內線——
    轉彎時 follower 會抄近路撞上 leader 的內側。麵包屑讓 follower 走 leader 走過的路。
    （docs/design/v2v_architecture.md §4 D6，網查坐實為標準做法。）

    每顆麵包屑存 (x, y, s)，s = 從第一顆起的累積弧長。沿軌跡間距就是兩個 s 相減，
    這比歐氏直線距離更能反映「跟車距離」（轉彎時直線距離會被抄近路低估）。

    ⚠ **最近點搜尋必須限制在上次投影點附近的弧長窗口內，不能全域搜尋。**
       8 字軌跡會自我交叉。follower 開到交叉點時，全域搜尋會把它投影到 leader
       半圈之前走過的**另一條分支**上，剩餘弧長瞬間暴增（2026-07-10 實測：真實跟車
       距離 1.20m，path_gap 卻算成 7.40m）-> follower 誤以為掉隊而暴衝。
       窗口法（progress-preserving projection）讓投影點只能沿軌跡緩慢前進，
       交叉點的另一分支在弧長上距離半個 8 字（≈8.9m），落在窗口外，自然被排除。
    """

    def __init__(self, min_spacing: float = 0.05, max_points: int = 20000,
                 search_back: float = 0.5, search_fwd: float = 5.0):
        self.min_spacing = min_spacing
        self.max_points = max_points
        self.search_back = search_back   # 投影點可往回搜的弧長 (m)
        self.search_fwd = search_fwd     # 投影點可往前搜的弧長 (m)，需 < 半個 8 字的弧長
        self.pts = []   # list of (x, y, s)
        self._cursor = 0   # 上次投影點索引；搜尋窗口以它為中心

    def __len__(self):
        return len(self.pts)

    @property
    def total_length(self) -> float:
        return self.pts[-1][2] if self.pts else 0.0

    def reset_cursor(self):
        self._cursor = 0

    def append(self, x: float, y: float) -> bool:
        """距離上一顆夠遠才記錄。回傳是否真的加入。"""
        if not self.pts:
            self.pts.append((x, y, 0.0))
            return True
        lx, ly, ls = self.pts[-1]
        d = math.hypot(x - lx, y - ly)
        if d < self.min_spacing:
            return False
        self.pts.append((x, y, ls + d))
        if len(self.pts) > self.max_points:
            self.pts.pop(0)
            self._cursor = max(0, self._cursor - 1)
        return True

    def closest_index(self, x: float, y: float) -> int:
        """離 (x,y) 最近的麵包屑索引，**只在 cursor 附近的弧長窗口內搜尋**。

        搜完會把 cursor 移到結果上，所以連續呼叫時投影點沿軌跡單調推進。
        """
        n = len(self.pts)
        if n == 0:
            return 0
        self._cursor = min(self._cursor, n - 1)
        s_cur = self.pts[self._cursor][2]

        lo = self._cursor
        while lo > 0 and self.pts[lo][2] > s_cur - self.search_back:
            lo -= 1
        hi = self._cursor
        while hi < n - 1 and self.pts[hi][2] < s_cur + self.search_fwd:
            hi += 1

        best_i, best_d2 = lo, float('inf')
        for i in range(lo, hi + 1):
            px, py, _ = self.pts[i]
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_i, best_d2 = i, d2
        self._cursor = best_i
        return best_i

    def path_gap(self, x: float, y: float) -> float:
        """從 (x,y) 沿軌跡到 leader 最新位置的跟車距離。

        ＝「(x,y) 到軌跡最近點的直線距離」＋「該點到軌跡末端的剩餘弧長」。

        ⚠ 前面那一項不能省。demo 開場時 follower 站在 leader **起點後方 GAP0 公尺**、
           根本還不在軌跡上，此時剩餘弧長是 0——只算弧長會謊報「已經貼上 leader」，
           follower 會以為要煞車。上了軌跡之後這一項自然趨近 0，退化成純弧長。
        """
        if not self.pts:
            return 0.0
        i = self.closest_index(x, y)
        px, py, s = self.pts[i]
        return math.hypot(x - px, y - py) + (self.total_length - s)

    def lookahead_point(self, x: float, y: float, lookahead: float):
        """從最近點沿軌跡往前走 lookahead 弧長，取該處的麵包屑當純追蹤目標。

        走到底就回最後一顆（leader 目前位置）。軌跡為空回 None。
        """
        if not self.pts:
            return None
        i = self.closest_index(x, y)
        s_target = self.pts[i][2] + lookahead
        for j in range(i, len(self.pts)):
            if self.pts[j][2] >= s_target:
                return self.pts[j][0], self.pts[j][1]
        return self.pts[-1][0], self.pts[-1][1]

    def prune_before(self, x: float, y: float, keep_back: float = 1.0):
        """丟掉 follower 已經走過很遠的舊麵包屑，避免線性掃描愈跑愈慢。

        keep_back：在投影點之後仍保留多長的弧長（保守留一段，防投影抖動跳點）。
        """
        if len(self.pts) < 2:
            return
        i = self.closest_index(x, y)
        s_keep = self.pts[i][2] - keep_back
        cut = 0
        for j in range(i):
            if self.pts[j][2] < s_keep:
                cut = j + 1
            else:
                break
        if cut:
            del self.pts[:cut]
            self._cursor = max(0, self._cursor - cut)


# ---------------------------------------------------------------------------
# V2/V3 驗收評分（純數學，供 score_follow 節點呼叫）
#
# 抽出來的目的：score_follow.py 是 ROS 節點（import rclpy），無法在筆電 pytest；
# 把「會算錯 / 會誤判 PASS」的部分集中在這裡，就能不靠 ROS/車/Pi 驗證評分邏輯。
# ---------------------------------------------------------------------------

def rms(vals) -> float:
    """均方根。空序列回 nan——呼叫端必須自行判斷樣本是否足夠，不得把 nan 當 0 判 PASS。"""
    vals = list(vals)
    return math.sqrt(sum(e * e for e in vals) / len(vals)) if vals else float('nan')


def point_segment_distance(px, py, ax, ay, bx, by) -> float:
    """點 (px,py) 到線段 AB 的最短距離。線段退化成一點時回點到點距離。"""
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    s = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + s * dx), py - (ay + s * dy))


def cross_track_series(follower_xy, leader_path):
    """每個 follower 點到 leader 折線的最短距離（全域最近段）。

    leader_path 少於 2 點時構不成折線，整段回 nan——**擋掉「只有 1 筆樣本時
    min() 對空序列丟 ValueError 直接崩潰」**，並讓呼叫端知道這段不可信（nan 不入 PASS）。
    """
    if len(leader_path) < 2:
        return [float('nan')] * len(follower_xy)
    out = []
    for px, py in follower_xy:
        best = min(point_segment_distance(px, py, ax, ay, bx, by)
                   for (ax, ay), (bx, by) in zip(leader_path, leader_path[1:]))
        out.append(best)
    return out


def score_following(rows, *, rms_limit, collision_dist, xte_limit, settle,
                    leader_length, min_samples=100, min_leader_length=1.0,
                    leader_moving_since=None):
    """把 follower 跟隨性能算成三判準與 PASS/FAIL。純函式，零 ROS 依賴。

    rows：list of (t, fx, fy, lx, ly, gap, err, center_dist)，lx/ly 已在 follower 座標系。
    回傳 dict，含各項指標與：
        code   0=PASS  1=FAIL  2=不予評分（樣本不足或 leader 沒動——絕不回 PASS）
        reason code=2 時的說明；其餘為空字串。

    兩道防呆（擋「假 PASS」，這是評分器最危險的失效模式）：
      1. 配對樣本數 < min_samples          -> code 2（幾筆資料也能算出漂亮 RMS 而誤判綠燈）
      2. leader 軌跡總長 < min_leader_length -> code 2（leader 根本沒動，跟隨無從評估）
    """
    n = len(rows)
    res = {'n_samples': n, 'code': 2, 'reason': '',
           'rms_all': float('nan'), 'rms_moving': float('nan'),
           'max_abs_err': float('nan'), 'min_center_dist': float('nan'),
           'xte_rms': float('nan'), 'xte_max': float('nan'),
           'xte': [], 'leader_length': leader_length,
           'pass_rms': False, 'pass_collision': False, 'pass_xte': False}

    if n < min_samples:
        res['reason'] = f'配對樣本不足（{n} < {min_samples}），資料太少不予評分'
        return res
    if leader_length < min_leader_length:
        res['reason'] = (f'leader 幾乎沒移動（軌跡總長 {leader_length:.2f} m '
                         f'< {min_leader_length:g} m），無法評估跟隨')
        return res

    all_err = [r[6] for r in rows]
    res['min_center_dist'] = min(r[7] for r in rows)
    res['rms_all'] = rms(all_err)
    res['max_abs_err'] = max(abs(e) for e in all_err)
    if leader_moving_since is not None:
        res['rms_moving'] = rms([r[6] for r in rows if r[0] >= leader_moving_since])

    follower_xy = [(r[1], r[2]) for r in rows]
    leader_path = [(r[3], r[4]) for r in rows]
    xte = cross_track_series(follower_xy, leader_path)
    res['xte'] = xte
    t_start = rows[0][0]
    xte_settled = [xte[i] for i, r in enumerate(rows)
                   if r[0] - t_start >= settle and not math.isnan(xte[i])]
    res['xte_rms'] = rms(xte_settled)
    res['xte_max'] = max(xte_settled) if xte_settled else float('nan')

    res['pass_rms'] = res['rms_all'] < rms_limit
    res['pass_collision'] = res['min_center_dist'] > collision_dist
    res['pass_xte'] = bool(xte_settled) and res['xte_max'] < xte_limit
    # cross-track 的前提是 follower 真的走在 leader 路徑上。leader 路徑若沒有比跟車
    # 距離長很多，follower 全程都在路徑起點後方等，此時 xte 量到的是「還沒上路」的
    # 幾何距離，不是切內線（2026-07-11 V3 煙霧測試：leader 僅 1.4m、xte 假性 0.65m）。
    # 只做誠實標註，不改判定——FAIL 仍是 FAIL，但報告要說清楚這條不可評。
    res['xte_evaluable'] = leader_length >= 2.0 * _xte_gap_ref(rows)
    res['code'] = 0 if (res['pass_rms'] and res['pass_collision'] and res['pass_xte']) else 1
    return res


def _xte_gap_ref(rows) -> float:
    """cross-track 可評性的參考尺度：全程間距的中位數（≈實際跟車距離）。"""
    gaps = sorted(r[5] for r in rows)
    return gaps[len(gaps) // 2] if gaps else 0.0


# ---------------------------------------------------------------------------
# D9：V2V 鏈路延遲/丟包量測（純數字狀態機，零 ROS 依賴）
#
# 設計依據：docs/design/v2v_architecture.md §5（D9）。
# relay 端每筆蓋「發送時刻」進 header.stamp、遞增序號塞進 twist.covariance；
# follower 端每收到一筆，把 (發送時刻, 收到時刻, 序號) 三個純量丟進 LinkMonitor.record()。
# 把統計邏輯抽到這裡的目的與評分器相同：follower.py 是 ROS 節點無法筆電 pytest，
# 但「延遲差／跳號累計」是純數字運算，抽出來就能不靠 ROS/車/Pi 驗證。
# ---------------------------------------------------------------------------

# 序號的載體：nav_msgs/Odometry 的 twist.covariance[此索引]。
#   為什麼借 covariance：ROS2 的 std_msgs/Header 已移除 ROS1 的 seq 欄位，Odometry 沒有
#   其他整數欄位可用；covariance 是 36 個 float64（可精確表示 2^53 內整數），而 v2v/state
#   這條專用通道的下游（follower / score_follow）都不讀 covariance，覆寫最安全、語義最乾淨。
#   ⚠ relay（寫）與 follower（讀）必須用同一個索引，故定義在此當單一真實來源。
V2V_SEQ_COV_INDEX = 0


class LinkMonitor:
    """V2V 鏈路延遲/丟包統計。純數字狀態機，逐筆餵入、收尾出統計。

    ⚠ 延遲的**絕對值**只有在兩車時鐘用 chrony 對齊後才有意義（V1 工作包）；sim 雙車
       同一時鐘域，延遲會接近 0。**丟包數／跳號在任何情況都有意義**（靠序號，與時鐘無關）。
    """

    def __init__(self):
        self.samples = []      # list of (seq, send_t, recv_t, delay, gap)
        self.last_seq = None   # 目前收過的最大序號（只前進，不被亂序拉回）
        self.lost = 0          # 累計丟包數（序號缺口總和）
        self.duplicated = 0    # 序號重複次數
        self.reordered = 0     # 序號倒退（亂序到達）次數

    def record(self, send_t: float, recv_t: float, seq) -> dict:
        """記一筆。回傳這筆的 {seq, delay, gap}（gap＝此筆之前偵測到的新增丟包數）。"""
        seq = int(seq)
        delay = recv_t - send_t
        gap = 0
        if self.last_seq is not None:
            step = seq - self.last_seq
            if step > 1:
                gap = step - 1        # 中間缺了 step-1 筆
                self.lost += gap
            elif step == 0:
                self.duplicated += 1
            elif step < 0:
                self.reordered += 1   # 亂序：晚到的舊序號
        # last_seq 只在前進時更新，避免亂序把基準拉回去、後續全被誤判成丟包
        if self.last_seq is None or seq > self.last_seq:
            self.last_seq = seq
        self.samples.append((seq, send_t, recv_t, delay, gap))
        return {'seq': seq, 'delay': delay, 'gap': gap}

    def summary(self) -> dict:
        """收尾統計。expected＝序號跨度（首筆到最大序號應含的筆數）；loss_rate＝lost/expected。"""
        n = len(self.samples)
        delays = [s[3] for s in self.samples]
        expected = 0
        if n >= 1 and self.last_seq is not None:
            first_seq = self.samples[0][0]
            expected = self.last_seq - first_seq + 1
        loss_rate = (self.lost / expected) if expected > 0 else float('nan')
        return {
            'n': n,                        # 實際收到筆數
            'expected': expected,          # 序號跨度應收到筆數
            'lost': self.lost,             # 丟包數
            'loss_rate': loss_rate,        # 丟包率
            'duplicated': self.duplicated,
            'reordered': self.reordered,
            'delay_mean': (sum(delays) / n) if n else float('nan'),
            'delay_rms': rms(delays),
            'delay_max': max(delays) if delays else float('nan'),
            'delay_min': min(delays) if delays else float('nan'),
        }

    def rows(self):
        """逐筆 (seq, send_t, recv_t, delay_ms, gap)，供寫 CSV。延遲轉成毫秒。"""
        return [(seq, send_t, recv_t, delay * 1000.0, gap)
                for (seq, send_t, recv_t, delay, gap) in self.samples]
