"""pytest 測試：protocol.CmdShaper（換向先停＋起步加速度上限，EX4 限速 2026-09-24）。

不依賴 ROS；可在 Windows 直接跑：
    cd ros2_ws/src/stm32_bridge && python -m pytest test/test_cmd_shaper.py -v

安全性質（整形器在急停鏈上，任何一條被破壞都可能讓車更晚停或更快）：
  S1 只會變小：|v_out| ≤ |v_in|、|w_out| ≤ |w_in|，且方向不會反過來
  S2 停車不延後：目標 0 → 當步輸出 0
  S3 減速不延後：同方向變慢 → 當步放行
"""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest  # noqa: E402

from stm32_bridge.protocol import CMD_SHAPER_MAX_DT_S, CmdShaper  # noqa: E402

DT = 0.05            # bridge 20 Hz 送指令
DWELL = 0.3
ACCEL = 0.5


def run(shaper, seq, t0=0.0, dt=DT):
    """seq＝[(v, w), ...]，每步間隔 dt；回傳 [(t, v_out, w_out), ...]。"""
    out = []
    for i, (v, w) in enumerate(seq):
        t = t0 + i * dt
        vo, wo = shaper.step(v, w, t)
        out.append((t, vo, wo))
    return out


def cruise(shaper, v, w=0.0, seconds=2.0):
    """先把整形器帶到穩定巡航 v。"""
    return run(shaper, [(v, w)] * int(seconds / DT))


def random_seq(rng, n=400):
    seq = []
    for _ in range(n):
        r = rng.random()
        if r < 0.15:
            seq.append((0.0, 0.0))
        else:
            seq.append((rng.uniform(-0.3, 0.3), rng.uniform(-2.0, 2.0)))
    return seq


# ---------------------------------------------------------------- 預設＝直通

def test_default_is_bit_identical_passthrough():
    rng = random.Random(1)
    seq = random_seq(rng)
    out = run(CmdShaper(), seq)
    assert [(v, w) for _, v, w in out] == seq


# ---------------------------------------------------------------- 安全性質

@pytest.mark.parametrize('dwell,accel', [(DWELL, 0.0), (0.0, ACCEL), (DWELL, ACCEL)])
def test_S1_never_larger_never_flipped(dwell, accel):
    rng = random.Random(2)
    seq = random_seq(rng, 2000)
    out = run(CmdShaper(dwell, accel), seq)
    for (v, w), (_, vo, wo) in zip(seq, out):
        assert abs(vo) <= abs(v) + 1e-12
        assert abs(wo) <= abs(w) + 1e-12
        assert vo * v >= 0.0, '輸出方向不得與目標相反'
        assert wo * w >= 0.0


@pytest.mark.parametrize('dwell,accel', [(DWELL, 0.0), (0.0, ACCEL), (DWELL, ACCEL)])
def test_S2_stop_is_immediate_from_any_state(dwell, accel):
    rng = random.Random(3)
    for _ in range(200):
        s = CmdShaper(dwell, accel)
        run(s, random_seq(rng, rng.randint(1, 60)))
        t = s._t + DT
        assert s.step(0.0, 0.0, t) == (0.0, 0.0)


def test_S3_slowing_down_is_immediate():
    s = CmdShaper(DWELL, ACCEL)
    cruise(s, 0.25, 0.3)
    t = s._t + DT
    assert s.step(0.10, 0.12, t) == (0.10, 0.12)


# ---------------------------------------------------------------- 換向先停

def test_reversal_holds_zero_for_dwell_then_releases():
    s = CmdShaper(DWELL, 0.0)
    cruise(s, 0.25)
    t_rev = s._t + DT
    out = run(s, [(-0.25, 0.0)] * 20, t0=t_rev)
    zero_until = t_rev + DWELL          # 第一個 0 在 t_rev 送出，停滿 DWELL 才放行
    for t, vo, _ in out:
        if t < zero_until - 1e-9:
            assert vo == 0.0, 't=%.2f 還在停留期卻送了 %.3f' % (t - t_rev, vo)
    assert any(vo < 0 for _, vo, _ in out), '停留期滿後必須放行反方向'
    first_neg = min(t for t, vo, _ in out if vo < 0)
    assert first_neg - t_rev >= DWELL - 1e-9


def test_reversal_via_explicit_zero_still_waits_full_dwell():
    """Nav2 自己先送一拍 0 再反向：停留期從「輸出歸零」算起，不能被那一拍 0 抵掉。"""
    s = CmdShaper(DWELL, 0.0)
    cruise(s, 0.25)
    t0 = s._t + DT
    s.step(0.0, 0.0, t0)                                   # 輸出歸零
    assert s.step(-0.25, 0.0, t0 + DT) == (0.0, 0.0)        # 0.05 s < dwell
    assert s.step(-0.25, 0.0, t0 + DWELL + 1e-6)[0] < 0      # 滿 dwell 放行


def test_no_dwell_when_starting_from_rest():
    s = CmdShaper(DWELL, 0.0)
    assert s.step(-0.2, 0.0, 0.0) == (-0.2, 0.0)


def test_no_dwell_when_resuming_same_direction():
    s = CmdShaper(DWELL, 0.0)
    cruise(s, 0.25)
    t = s._t + DT
    s.step(0.0, 0.0, t)
    assert s.step(0.25, 0.0, t + DT) == (0.25, 0.0)


def test_dwell_sends_zero_angular_too():
    """停留期 w 也要 0：韌體要兩輪目標都是 0 才會把 PID 歸零。"""
    s = CmdShaper(DWELL, 0.0)
    cruise(s, 0.25, 0.5)
    t = s._t + DT
    assert s.step(-0.25, 0.8, t) == (0.0, 0.0)


# ---------------------------------------------------------------- 加速度上限

def test_accel_limit_from_rest():
    s = CmdShaper(0.0, ACCEL)
    out = run(s, [(0.25, 0.0)] * 30)
    prev = 0.0
    for _, vo, _ in out:
        assert vo - prev <= ACCEL * DT + 1e-12
        prev = vo
    assert out[-1][1] == pytest.approx(0.25)
    t_reach = min(t for t, vo, _ in out if vo >= 0.25 - 1e-12)
    assert t_reach == pytest.approx(0.25 / ACCEL + DT, abs=DT + 1e-9)


def test_accel_limit_preserves_curvature():
    s = CmdShaper(0.0, ACCEL)
    for _, vo, wo in run(s, [(0.25, 0.6)] * 10)[1:]:
        assert wo / vo == pytest.approx(0.6 / 0.25)


def test_timer_stall_does_not_release_a_big_jump():
    s = CmdShaper(0.0, ACCEL)
    s.step(0.0, 0.0, 0.0)
    vo, _ = s.step(0.25, 0.0, 5.0)          # 計時器卡了 5 秒
    assert vo <= ACCEL * CMD_SHAPER_MAX_DT_S + 1e-12


def test_reversal_then_accel_ramps_from_zero():
    s = CmdShaper(DWELL, ACCEL)
    cruise(s, 0.25)
    t_rev = s._t + DT
    out = run(s, [(-0.25, 0.0)] * 30, t0=t_rev)
    negs = [vo for _, vo, _ in out if vo < 0]
    assert negs and abs(negs[0]) <= ACCEL * DT + 1e-12, '反向放行後也要從 0 慢慢加速'
