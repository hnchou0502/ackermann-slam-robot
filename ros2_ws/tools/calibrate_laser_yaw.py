#!/usr/bin/env python3
"""差分定位光達零度朝向：比對「有障礙物」與「無障礙物」兩次掃描，
距離變近最多的那個方向＝障礙物所在，據此反推 laser_yaw。

不受環境雜亂影響——只看「變化量」，靜態雜物會被抵銷掉。

用法：
  python3 calibrate_laser_yaw.py base   # 障礙物「已移開」時跑，存基準
  python3 calibrate_laser_yaw.py test   # 障礙物「放在車頭正前方」時跑，比對並給答案
  # 之後用 verify_laser_tf.py 做端到端複驗（確認經真實 TF 換算後落在車頭正前方）

實績：2026-07-20 car1 C1 用此法定出 laser_yaw=180°，訊號 0.551 m vs 雜訊 0.007 m（79 倍）。
⚠ 不可改用「找最近點」——當時 ±180° 與 -30° 各有雜物，單看最近點分不出來。
"""
import sys
import json
import math
import statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MODE = sys.argv[1] if len(sys.argv) > 1 else "base"
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
STORE = "/tmp/yaw_base.json"
SECTOR = 5.0  # 度，比剖面細，定位更準
NSEC = int(360 / SECTOR)


class P(Node):
    def __init__(self):
        super().__init__("yaw_diff")
        self.msgs = []
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.msgs.append(msg)


def collect():
    rclpy.init()
    node = P()
    t0 = node.get_clock().now()
    while (node.get_clock().now() - t0).nanoseconds / 1e9 < DUR:
        rclpy.spin_once(node, timeout_sec=0.1)
    msgs = node.msgs
    rclpy.shutdown()
    if not msgs:
        return None

    per_frame = []
    for m in msgs:
        buckets = [[] for _ in range(NSEC)]
        for i, r in enumerate(m.ranges):
            if math.isinf(r) or math.isnan(r) or not (m.range_min <= r <= m.range_max):
                continue
            ang = math.degrees(m.angle_min + i * m.angle_increment)
            idx = int(((ang + 180.0) % 360.0) / SECTOR) % NSEC
            buckets[idx].append(r)
        per_frame.append([min(b) if b else None for b in buckets])

    out = []
    for k in range(NSEC):
        vals = [f[k] for f in per_frame if f[k] is not None]
        out.append(statistics.median(vals) if vals else None)
    print(f"（採樣 {len(msgs)} 幀）")
    return out


def ang_of(k):
    return -180.0 + k * SECTOR + SECTOR / 2.0


def main():
    prof = collect()
    if prof is None:
        print("沒收到 /scan")
        return 1

    if MODE == "base":
        with open(STORE, "w") as f:
            json.dump(prof, f)
        n = sum(1 for x in prof if x is not None)
        print(f"基準已存（{n}/{NSEC} 個扇區有回波）→ {STORE}")
        print("現在把障礙物放到車頭正前方，再跑：python3 yaw_diff.py test")
        return 0

    with open(STORE) as f:
        base = json.load(f)

    deltas = []
    for k in range(NSEC):
        b, t = base[k], prof[k]
        if b is None or t is None:
            continue
        deltas.append((k, b - t, b, t))  # 正值＝變近了

    if not deltas:
        print("兩次掃描沒有可比對的扇區")
        return 1

    deltas.sort(key=lambda x: -x[1])
    print(f"\n{'角度':>8}  {'基準':>7}  {'現在':>7}  {'變近':>7}")
    for k, d, b, t in deltas[:8]:
        print(f"{ang_of(k):+8.1f}°  {b:7.3f}  {t:7.3f}  {d:+7.3f} m")

    k, d, b, t = deltas[0]
    theta = ang_of(k)
    if d < 0.10:
        print(f"\n⚠ 最大變化只有 {d:.3f} m，太小——障礙物可能沒放好，或兩次之間環境沒變。")
        print("   請確認障礙物有確實移開/放回，且放在光達掃描高度上。")
        return 1

    yaw = -math.radians(theta)
    # 正規化到 -pi ~ pi
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    print(f"\n障礙物出現在 laser_frame 的 {theta:+.1f}°（距離變近 {d:.3f} m）")
    print(f"→ 該方向就是車頭正前方，故 laser_yaw = {-theta:+.1f}° = {yaw:+.6f} rad")
    for name, val in (("0（零度朝正前）", 0.0), ("180（零度朝車尾）", math.pi),
                      ("+90", math.pi / 2), ("-90", -math.pi / 2)):
        if abs(abs(yaw) - abs(val)) < math.radians(8):
            print(f"   ≈ 標準值 {name}")
            break
    return 0


if __name__ == "__main__":
    sys.exit(main())
