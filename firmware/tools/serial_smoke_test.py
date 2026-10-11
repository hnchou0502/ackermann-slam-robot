#!/usr/bin/env python3
"""STM32 韌體串列台測工具（Windows 直連 CH340 用）。

用途：新車開箱驗收（car2 bring-up）、燒錄後煙霧測試。車輪必須架空。
協定：下行 $CMD,vx,wz,#（m/s、rad/s ×1000 整數）；上行 $FBK,vL,vR,pwmL,pwmR,gyroZ,yaw,#

用法：
  python serial_smoke_test.py --port COM3 passive 5
      被動聽 FBK 5 秒：驗頻率 ~25Hz、欄位數、靜止陀螺 bias
  python serial_smoke_test.py --port COM3 drive 0.2 0 3
      以 20Hz 連送 vx=0.2 m/s、wz=0 共 3 秒後送停；印 vL/vR/pwm 統計
  python serial_smoke_test.py --port COM3 servo
      靜止打角測試：wz=+0.5（前輪應左偏）2 秒 → wz=-0.5（右偏）2 秒 → 停
"""
import argparse
import statistics
import sys
import time

import serial

sys.stdout.reconfigure(encoding="utf-8")  # cp950 主控台防炸


def parse_fbk(line: str):
    # $FBK,vL,vR,pwmL,pwmR,gyroZ,yaw,#
    if not line.startswith("$FBK"):
        return None
    parts = line.split(",")
    if len(parts) < 7:
        return None
    try:
        return [int(x) for x in parts[1:7]]
    except ValueError:
        return None


def collect(ser, seconds: float, label: str):
    rows, bad, t0 = [], 0, time.time()
    buf = b""
    while time.time() - t0 < seconds:
        buf += ser.read(256)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            r = parse_fbk(line)
            if r is not None:
                rows.append(r)
            elif line.startswith("$FBK"):
                bad += 1
    dt = time.time() - t0
    print(f"[{label}] {len(rows)} 筆 FBK / {dt:.1f}s = {len(rows)/dt:.1f} Hz（壞行 {bad}）")
    return rows


def summarize(rows, expect_v=None):
    if not rows:
        print("  !! 沒收到任何 FBK — 檢查埠、線、供電")
        return
    vl = [r[0] for r in rows]
    vr = [r[1] for r in rows]
    pl = [r[2] for r in rows]
    pr = [r[3] for r in rows]
    gz = [r[4] / 1000.0 for r in rows]  # deg/s
    yaw = rows[-1][5] / 1000.0
    print(f"  vL 平均 {statistics.mean(vl):7.1f} mm/s   vR 平均 {statistics.mean(vr):7.1f} mm/s")
    print(f"  pwmL 平均 {statistics.mean(pl):7.0f}      pwmR 平均 {statistics.mean(pr):7.0f}")
    print(f"  gyroZ 平均 {statistics.mean(gz):+.3f} deg/s（靜止應在 ±0.2 內）  yaw 目前 {yaw:+.2f} deg")
    if expect_v is not None:
        mm = expect_v * 1000
        ok_l = abs(statistics.mean(vl) - mm) < abs(mm) * 0.25 + 30
        ok_r = abs(statistics.mean(vr) - mm) < abs(mm) * 0.25 + 30
        print(f"  目標 {mm:.0f} mm/s → 左輪 {'PASS' if ok_l else 'FAIL'}／右輪 {'PASS' if ok_r else 'FAIL'}"
              f"（判準：誤差 <25%+30mm/s；反轉/不轉都會 FAIL）")


def send_cmd(ser, vx: float, wz: float):
    ser.write(f"$CMD,{int(vx*1000)},{int(wz*1000)},#\n".encode())


def stream(ser, vx, wz, seconds, label, expect_v=None):
    print(f"[{label}] 送 vx={vx} m/s, wz={wz} rad/s 共 {seconds}s（20Hz 連送）")
    rows, t0 = [], time.time()
    buf = b""
    next_tx = 0.0
    while time.time() - t0 < seconds:
        now = time.time()
        if now >= next_tx:
            send_cmd(ser, vx, wz)
            next_tx = now + 0.05
        buf += ser.read(64)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            r = parse_fbk(raw.decode(errors="replace").strip())
            if r is not None:
                rows.append(r)
    send_cmd(ser, 0, 0)
    # 跳過前 0.5s 起步暫態再統計
    steady = rows[len(rows) // 6:] if len(rows) > 12 else rows
    summarize(steady, expect_v=expect_v)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("mode", choices=["passive", "drive", "servo"])
    ap.add_argument("args", nargs="*", type=float)
    a = ap.parse_args()

    ser = serial.Serial(a.port, 115200, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    if a.mode == "passive":
        secs = a.args[0] if a.args else 5
        summarize(collect(ser, secs, "passive"))
    elif a.mode == "drive":
        vx = a.args[0] if a.args else 0.2
        wz = a.args[1] if len(a.args) > 1 else 0.0
        secs = a.args[2] if len(a.args) > 2 else 3
        stream(ser, vx, wz, secs, "drive", expect_v=vx)
        print("[stop] 已送停車指令；韌體看門狗 500ms 亦會自動停")
    elif a.mode == "servo":
        stream(ser, 0, 0.5, 2, "servo-left（前輪應左偏）")
        stream(ser, 0, -0.5, 2, "servo-right（前輪應右偏）")
        send_cmd(ser, 0, 0)
        print("[stop] 打角測試結束，前輪應自動回正（v74 行為）")
    ser.close()


if __name__ == "__main__":
    main()
