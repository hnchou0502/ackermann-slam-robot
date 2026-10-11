"""STM32 UART 協定：純函式模組，零 ROS 依賴。

支援兩種 $FBK 格式（向後相容）：
    舊 5 欄: $FBK,vL,vR,pwmL,pwmR,#
    新 7 欄: $FBK,vL,vR,pwmL,pwmR,gyroZ,yaw,#

回傳值的 SI 單位：
    vl / vr   : m/s（整數 mm/s ÷ 1000）
    pwml/pwmr : 原始整數（無單位轉換，STM32 duty count）
    gyro_z    : rad/s（整數 deg/s×1000 ÷ 1000 × π/180）
    yaw       : rad（整數 deg×1000 ÷ 1000 × π/180）
"""

import math

DEG2RAD = math.pi / 180.0


def parse_fbk(line: str):
    """解析一行 $FBK 封包，回傳 dict 或 None（非法/不足欄位時）。

    邏輯與 bridge_node.py handle_line() 完全對應：
      - 從 line 中找到 '$FBK' 起始位置（允許前置雜訊）
      - 去掉 '#'、以 ',' 分割
      - parts[1]=vL, parts[2]=vR, parts[3]=pwmL, parts[4]=pwmR
      - parts[5]=gyroZ (deg/s×1000), parts[6]=yaw (deg×1000)  ← 新格式才有
    """
    if '$FBK' not in line:
        return None
    line = line[line.find('$FBK'):]
    parts = line.replace('#', '').split(',')
    if len(parts) < 5:
        return None
    try:
        v_l = int(parts[1]) / 1000.0   # mm/s -> m/s
        v_r = int(parts[2]) / 1000.0
    except ValueError:
        return None

    result = {
        'vl': v_l,
        'vr': v_r,
        'pwml': None,
        'pwmr': None,
        'gyro_z': None,
        'yaw': None,
    }

    # pwmL / pwmR：讀取但不強制（解析失敗不視為整筆非法，與 bridge_node 相同）
    try:
        result['pwml'] = int(parts[3])
        result['pwmr'] = int(parts[4])
    except ValueError:
        pass

    # 新格式：parts[5]=gyroZ, parts[6]=yaw
    if len(parts) >= 7:
        try:
            result['gyro_z'] = int(parts[5]) / 1000.0 * DEG2RAD   # deg/s -> rad/s
            result['yaw'] = int(parts[6]) / 1000.0 * DEG2RAD       # deg -> rad
        except ValueError:
            pass   # 格式有但值非法 -> 保持 None，與 bridge_node 的 except ValueError: pass 行為相同

    return result


def format_cmd(vx: float, wz: float) -> str:
    """格式化 $CMD 送車字串，與 bridge_node.py send_cmd() 逐字元相同。

    回傳含換行的完整字串（'\\n' 結尾），例如：
        '$CMD,500,-200,#\\n'
    """
    return f'$CMD,{int(vx * 1000)},{int(wz * 1000)},#\n'


# ---------------------------------------------------------------------------
# 序列埠健康判定（EX3，2026-08-11）
# ---------------------------------------------------------------------------

# 連續失敗超過這麼久就放棄。2 秒足以濾掉偶發的一兩次讀寫錯誤，
# 又不會讓「線被拔掉」的狀態拖太久（拖著的代價見下方 docstring）。
SERIAL_DEAD_AFTER_S = 2.0


def serial_should_abort(bad_since, now, timeout=SERIAL_DEAD_AFTER_S):
    """序列埠持續失敗到該放棄了嗎？bad_since=None 代表目前正常。

    **為什麼要主動放棄，而不是一直重試**（2026-08-11 EX3 實測發現）：
    拔掉 STM32 的 USB 線之後，韌體的 watchdog 會讓車停下來（硬體兜底成立），
    但 bridge 只是不斷噴讀寫錯誤、**不會退出**，於是 Nav2 和 explorer 全都還活著。
    這時系統處於殭屍狀態：車停了，但整套控制堆疊還在跑——**只要有人把線插回去，
    車可能立刻又動起來**，而現場最自然的反應就是「拔線 → 檢查 → 插回去」。

    正確行為＝bridge 主動結束行程，觸發 supervisor 的 OnProcessExit 把整串關掉。
    """
    if bad_since is None:
        return False
    return (now - bad_since) >= timeout


# ---------------------------------------------------------------------------
# 停車確認的連續計數（EX3；Codex R3-F1，2026-08-11）
# ---------------------------------------------------------------------------

STOPPED_SPEED_MPS = 0.02     # 輪速低於此值視為已停
CONFIRM_FRAMES = 2           # 要連續這麼多筆**各自獨立**的新 FBK 都低於門檻才算確認


def advance_stop_confirmation(speeds, consecutive, threshold=STOPPED_SPEED_MPS,
                              need=CONFIRM_FRAMES):
    """用「本輪新解析到的速度清單」推進連續計數。回傳 (新的 consecutive, 是否確認)。

    **每筆 FBK 只能被計一次**（Codex R3-F1）：舊版用「最後一筆時戳 > t0」當作
    「有新資料」，但那個條件在序列埠靜默後仍然成立——同一筆低速資料會在後續每一輪
    迴圈重複遞增計數，一筆就湊出「連續 2 筆」。所以本函式只吃「本輪實際解析到的
    那幾筆」，沒有解析到就傳空清單，計數原地不動。

    這是同一類錯誤的第三種形式（把推論當量測 → 把舊資料當量測 → 把同一筆當兩筆），
    因此把判定抽成純函式並用測試釘死。
    """
    # ⚠ **必須把整批跑完才判定**（Codex R5-F1）：舊版一湊滿 need 就立刻 return，
    #   會忽略同一批裡**後面更新的**回授。反例 speeds=[0.0, 0.0, 0.15]：前兩筆就
    #   宣告確認，無視最新的 0.15 m/s，呼叫端還用 speeds[-1] 當 last_speed，
    #   於是印出「0.150 m/s < 0.020」這種自相矛盾的訊息。
    #   這是同一個毛病的第四種形式——把「批次中間的狀態」當成「批次結束的狀態」。
    for sp in speeds:
        if sp < threshold:
            consecutive += 1
        else:
            consecutive = 0      # 中間出現高速就重新累積
    return consecutive, consecutive >= need


# ---------------------------------------------------------------------------
# 指令整形：換向先停＋起步加速度上限（EX4 限速，2026-09-24）
# ---------------------------------------------------------------------------

CMD_SHAPER_MAX_DT_S = 0.1
"""單步最多採計的時間。計時器若卡頓 5 秒，不能因此一步放行 5 秒份的加速度。"""

CMD_ZERO_EPS = 1e-6


def _sign(x):
    return (x > CMD_ZERO_EPS) - (x < -CMD_ZERO_EPS)


class CmdShaper:
    """把要送給韌體的 (v, w) 整形，壓掉韌體增量式 PID 的過衝。

    **為什麼**（2026-09-24 EX4 run5 bag 實測）：實際輪速最高 0.474 m/s，但橋接夾限後
    送到韌體的指令換算單輪最高只有 0.290。超過 0.28 的 931 筆裡 **58% 發生在
    前進↔倒車換向後 2 秒內**。韌體只在「目標＝0」時把 PID 整段歸零；Nav2 常直接從
    +0.25 跳到 −0.25，中間不經過 0，累積的舊輸出沒清掉 → 換向那一下衝過頭。

    兩道整形，**都只會讓指令變小、不會變大，也不會延後停車**：

      1. 換向先停 `reversal_dwell_s`：要反向時先送 (0, 0)，並維持到「輸出歸零後
         已滿 dwell 秒」才放行反方向——讓韌體拿到目標 0、把 PID 歸零重來。
         0.3 s 的依據：EX3 實測從行進到 FBK 歸零 0.20–0.25 s（docs/design/ex3_fault_injection.md）。
      2. 起步加速度上限 `max_accel`（m/s²）：只限制「速度大小增加」；減速、停車一律
         當步放行。被限速時 w 等比例縮小，**保持轉彎曲率不變**（只是開慢一點，不會改路線）。

    兩者皆 0＝完全直通（與加入本類別前逐位元相同），這是預設。
    """

    def __init__(self, reversal_dwell_s=0.0, max_accel=0.0):
        self.dwell = float(reversal_dwell_s)
        self.max_accel = float(max_accel)
        self._t = None
        self._v = 0.0              # 上一步的輸出
        self._last_dir = 0         # 最後一次非零輸出的方向（+1／−1；0＝還沒動過）
        self._zero_since = None    # 輸出從非零變成零的時刻

    @property
    def enabled(self):
        return self.dwell > 0.0 or self.max_accel > 0.0

    def step(self, v, w, now):
        """吃「本步想送的 (v, w)」與現在時刻（秒），回傳實際要送的 (v, w)。"""
        dt = 0.0 if self._t is None else min(max(0.0, now - self._t), CMD_SHAPER_MAX_DT_S)
        self._t = now
        if not self.enabled:
            return self._emit(v, w, now)

        want = _sign(v)
        if self.dwell > 0.0 and want != 0:
            # 還在往反方向動 → 先停
            if _sign(self._v) == -want:
                return self._emit(0.0, 0.0, now)
            # 已經停了，但停下來的時間還不夠、且上一段是反方向 → 繼續停
            if (_sign(self._v) == 0 and self._last_dir == -want
                    and self._zero_since is not None
                    and now - self._zero_since < self.dwell):
                return self._emit(0.0, 0.0, now)

        if self.max_accel > 0.0 and abs(v) > abs(self._v):
            limit = abs(self._v) + self.max_accel * dt
            if abs(v) > limit:
                k = limit / abs(v)
                v, w = v * k, w * k
        return self._emit(v, w, now)

    def _emit(self, v, w, now):
        if _sign(v) == 0:
            if _sign(self._v) != 0:
                self._zero_since = now
        else:
            self._zero_since = None
            self._last_dir = _sign(v)
        self._v = v
        return v, w
