"""pytest 測試：stm32_bridge.protocol 的純函式邏輯。

不依賴 ROS；可在 Windows 直接跑：
    cd ros2_ws/src/stm32_bridge && python -m pytest test/test_protocol.py -v
"""
import math
import sys
import os

# 讓 pytest 找得到 stm32_bridge 套件（不走 ament install）
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from stm32_bridge.protocol import parse_fbk, format_cmd

DEG2RAD = math.pi / 180.0

# ── parse_fbk ──────────────────────────────────────────────────────────────


def test_fbk_old_format_basic():
    """舊 5 欄格式：vL vR 正確轉換成 m/s。"""
    result = parse_fbk('$FBK,100,200,1500,1600,#')
    assert result is not None
    assert abs(result['vl'] - 0.1) < 1e-9
    assert abs(result['vr'] - 0.2) < 1e-9


def test_fbk_old_format_imu_fields_are_none():
    """舊格式不含 IMU → gyro_z 與 yaw 應為 None。"""
    result = parse_fbk('$FBK,100,200,1500,1600,#')
    assert result['gyro_z'] is None
    assert result['yaw'] is None


def test_fbk_new_format_imu_values():
    """新 7 欄格式：gyroZ 與 yaw 正確換算成 rad/s 和 rad。"""
    # gyroZ=30000 → 30 deg/s；yaw=45000 → 45 deg
    result = parse_fbk('$FBK,300,400,1500,1600,30000,45000,#')
    assert result is not None
    assert abs(result['gyro_z'] - 30.0 * DEG2RAD) < 1e-9
    assert abs(result['yaw']    - 45.0 * DEG2RAD) < 1e-9


def test_fbk_new_format_velocity():
    """新格式中的 vl/vr 換算與舊格式相同。"""
    result = parse_fbk('$FBK,300,400,1500,1600,30000,45000,#')
    assert abs(result['vl'] - 0.3) < 1e-9
    assert abs(result['vr'] - 0.4) < 1e-9


def test_fbk_too_few_fields_returns_none():
    """欄位不足 5 個應回 None。"""
    assert parse_fbk('$FBK,100,200,#') is None
    assert parse_fbk('$FBK,100,#') is None
    assert parse_fbk('$FBK,#') is None


def test_fbk_empty_string_returns_none():
    """空字串應回 None。"""
    assert parse_fbk('') is None


def test_fbk_no_fbk_marker_returns_none():
    """不含 $FBK 的行應回 None。"""
    assert parse_fbk('$CMD,100,200,#') is None
    assert parse_fbk('hello world') is None


def test_fbk_leading_noise_stripped():
    """$FBK 前有雜訊（如 UART 溢位殘字）仍能解析。"""
    result = parse_fbk('GARBAGE_BYTES$FBK,500,-300,1000,2000,#')
    assert result is not None
    assert abs(result['vl'] - 0.5) < 1e-9
    assert abs(result['vr'] - (-0.3)) < 1e-9


def test_fbk_negative_velocity():
    """負速度（後退）正確轉換。"""
    result = parse_fbk('$FBK,-500,-300,1000,2000,#')
    assert abs(result['vl'] - (-0.5)) < 1e-9
    assert abs(result['vr'] - (-0.3)) < 1e-9


def test_fbk_zero_velocity():
    """零速度正確轉換。"""
    result = parse_fbk('$FBK,0,0,0,0,#')
    assert result is not None
    assert result['vl'] == 0.0
    assert result['vr'] == 0.0


def test_fbk_negative_imu():
    """新格式中 gyroZ/yaw 為負值（左轉）正確換算。"""
    result = parse_fbk('$FBK,100,100,1500,1500,-15000,-90000,#')
    assert abs(result['gyro_z'] - (-15.0 * DEG2RAD)) < 1e-9
    assert abs(result['yaw']    - (-90.0 * DEG2RAD)) < 1e-9


def test_fbk_invalid_velocity_value_returns_none():
    """vL/vR 欄位非整數時整筆回 None（對應 bridge_node 的 except ValueError: return）。"""
    assert parse_fbk('$FBK,abc,200,1500,1600,#') is None
    assert parse_fbk('$FBK,100,xyz,1500,1600,#') is None


# ── format_cmd ─────────────────────────────────────────────────────────────


def test_cmd_basic_format():
    """基本正速度格式正確，含換行。"""
    assert format_cmd(0.5, 0.2) == '$CMD,500,200,#\n'


def test_cmd_negative_values():
    """負值（後退+右轉）格式正確。"""
    assert format_cmd(-0.3, -1.0) == '$CMD,-300,-1000,#\n'


def test_cmd_zero():
    """停車指令格式正確。"""
    assert format_cmd(0.0, 0.0) == '$CMD,0,0,#\n'


def test_cmd_truncation_not_rounding():
    """int() 是截斷不是四捨五入。
    vx=0.9999 -> int(999.9) = 999（不是 1000）。
    """
    assert format_cmd(0.9999, 0.0) == '$CMD,999,0,#\n'


def test_cmd_negative_truncation():
    """負數截斷方向：int(-0.9999 * 1000) = int(-999.9) = -999。"""
    assert format_cmd(-0.9999, 0.0) == '$CMD,-999,0,#\n'


def test_cmd_ends_with_newline():
    """回傳字串必須以 '\\n' 結尾（韌體 strchr 靠換行判斷封包結束）。"""
    result = format_cmd(0.1, 0.0)
    assert result.endswith('\n')


def test_cmd_starts_with_dollar_cmd():
    """回傳字串開頭固定為 '$CMD,'。"""
    result = format_cmd(0.2, -0.5)
    assert result.startswith('$CMD,')


# --- 序列埠健康判定（EX3，2026-08-11）---

def test_serial_ok_when_never_failed():
    from stm32_bridge.protocol import serial_should_abort
    assert serial_should_abort(None, now=999.0) is False


def test_serial_tolerates_brief_failures():
    """偶發一兩次讀寫錯誤不該讓整套堆疊關閉。"""
    from stm32_bridge.protocol import serial_should_abort
    assert serial_should_abort(bad_since=100.0, now=101.9) is False


def test_serial_aborts_after_sustained_failure():
    """線被拔掉＝持續失敗 → 必須放棄，否則系統變殭屍（插回線可能突然開動）。"""
    from stm32_bridge.protocol import serial_should_abort
    assert serial_should_abort(bad_since=100.0, now=102.0) is True
    assert serial_should_abort(bad_since=100.0, now=105.0) is True


def test_serial_abort_threshold_is_locked():
    from stm32_bridge.protocol import SERIAL_DEAD_AFTER_S
    assert SERIAL_DEAD_AFTER_S == 2.0, \
        '改門檻要連同理由：太短會被偶發錯誤誤觸發，太長會讓殭屍狀態拖著'


# --- 停車確認的連續計數（Codex R3-F1，2026-08-11）---

def test_one_low_frame_then_silence_does_not_confirm():
    """只收到一筆低速、之後序列埠靜默 → 絕不能宣告確認。

    這正是 R3-F1 的反例：舊版用「最後時戳 > t0」判斷有無新資料，靜默後條件仍成立，
    同一筆會被重複計成「連續 2 筆」。
    """
    from stm32_bridge.protocol import advance_stop_confirmation
    c, done = advance_stop_confirmation([0.0], 0)
    assert (c, done) == (1, False)
    for _ in range(50):                       # 之後每輪都沒有新資料
        c, done = advance_stop_confirmation([], c)
        assert not done, '沒有新資料時計數不得推進'
    assert c == 1


def test_two_independent_low_frames_confirm():
    from stm32_bridge.protocol import advance_stop_confirmation
    c, done = advance_stop_confirmation([0.01], 0)
    assert not done
    c, done = advance_stop_confirmation([0.005], c)
    assert done, '兩筆各自獨立的低速回授才算確認'


def test_low_high_low_resets_the_streak():
    """中間出現高速代表車又動了，計數必須重新累積。"""
    from stm32_bridge.protocol import advance_stop_confirmation
    c, done = advance_stop_confirmation([0.01], 0)
    assert (c, done) == (1, False)
    c, done = advance_stop_confirmation([0.15], c)
    assert (c, done) == (0, False)
    c, done = advance_stop_confirmation([0.01], c)
    assert (c, done) == (1, False), '高速之後要從頭數起'


def test_two_low_frames_in_one_batch_confirm():
    """同一次讀取解析出兩筆低速也算兩筆（它們是獨立的 FBK 封包）。"""
    from stm32_bridge.protocol import advance_stop_confirmation
    c, done = advance_stop_confirmation([0.01, 0.008], 0)
    assert done


def test_stop_confirmation_constants_are_locked():
    from stm32_bridge.protocol import CONFIRM_FRAMES, STOPPED_SPEED_MPS
    assert CONFIRM_FRAMES == 2
    assert STOPPED_SPEED_MPS == 0.02


def test_high_speed_later_in_same_batch_cancels_confirmation():
    """同一批裡後面出現高速 → 不得確認（Codex R5-F1）。

    舊版一湊滿兩筆就 return，忽略同批更新的資料，還會印出
    「0.150 m/s < 0.020」這種自相矛盾的訊息。
    """
    from stm32_bridge.protocol import advance_stop_confirmation
    c, done = advance_stop_confirmation([0.0, 0.0, 0.15], 0)
    assert not done, '批次最後是高速，代表車還在動'
    assert c == 0


def test_batch_ending_with_two_lows_confirms():
    """高速之後又連續兩筆低速，批次結束時仍是停止狀態 → 確認。"""
    from stm32_bridge.protocol import advance_stop_confirmation
    c, done = advance_stop_confirmation([0.0, 0.0, 0.15, 0.0, 0.0], 0)
    assert done
    assert c == 2
