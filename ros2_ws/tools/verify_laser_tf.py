#!/usr/bin/env python3
"""端到端驗證 laser_yaw：用差分找出障礙物，再經真實 TF 換算到 base_link，
確認它落在車頭正前方（x>0、y≈0、方位角≈0°）。

比對基準＝/tmp/yaw_base.json（由 calibrate_laser_yaw.py base 產生，障礙物移開時錄的）。
不是驗「我設的參數等於我算的值」，而是驗「實體物件在車體座標系的位置對不對」。

實績：2026-07-20 car1 C1 設定 laser_yaw=180° 後，正前箱子落在
base_link x=+0.973 / y=+0.034 / 方位角 +2.0°，三判準全 PASS。
"""
import sys
import json
import math
import statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

DUR = 5.0
SECTOR = 5.0
NSEC = int(360 / SECTOR)
STORE = "/tmp/yaw_base.json"


class V(Node):
    def __init__(self):
        super().__init__("verify_tf")
        self.msgs = []
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.msgs.append(msg)


def main():
    rclpy.init()
    node = V()
    t0 = node.get_clock().now()
    while (node.get_clock().now() - t0).nanoseconds / 1e9 < DUR:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.msgs:
        print("沒收到 /scan")
        rclpy.shutdown()
        return 1

    # 1) 差分找障礙物所在的 laser_frame 角度
    per_frame = []
    for m in node.msgs:
        buckets = [[] for _ in range(NSEC)]
        for i, r in enumerate(m.ranges):
            if math.isinf(r) or math.isnan(r) or not (m.range_min <= r <= m.range_max):
                continue
            ang = math.degrees(m.angle_min + i * m.angle_increment)
            idx = int(((ang + 180.0) % 360.0) / SECTOR) % NSEC
            buckets[idx].append(r)
        per_frame.append([min(b) if b else None for b in buckets])

    cur = []
    for k in range(NSEC):
        vals = [f[k] for f in per_frame if f[k] is not None]
        cur.append(statistics.median(vals) if vals else None)

    with open(STORE) as f:
        base = json.load(f)

    best = None
    for k in range(NSEC):
        if base[k] is None or cur[k] is None:
            continue
        d = base[k] - cur[k]
        if best is None or d > best[1]:
            best = (k, d)

    # 防呆（Codex F5，2026-07-21）：無可比扇區、或變化量太小（沒放障礙物/放錯高度）
    # 都要明確失敗，不能把雜訊當障礙物往下算。門檻同 calibrate_laser_yaw 的 0.10 m。
    if best is None:
        print("兩次掃描沒有可比對的扇區——基準檔或現場掃描有問題")
        rclpy.shutdown()
        return 1
    k, drop = best
    if drop < 0.10:
        print(f"最大變化只有 {drop:.3f} m（<0.10）——障礙物可能沒放好或不在掃描高度，"
              f"不足以驗證，判定失敗")
        rclpy.shutdown()
        return 1
    theta_deg = -180.0 + k * SECTOR + SECTOR / 2.0
    rng = cur[k]
    print(f"障礙物：laser_frame {theta_deg:+.1f}°、距離 {rng:.3f} m（比基準近 {drop:.3f} m）")

    # 2) 用真實 TF 換算到 base_link
    try:
        tf = node.buf.lookup_transform(
            "base_link", node.msgs[-1].header.frame_id, rclpy.time.Time())
    except Exception as e:
        print(f"查不到 TF：{e}")
        rclpy.shutdown()
        return 1

    th = math.radians(theta_deg)
    lx, ly = rng * math.cos(th), rng * math.sin(th)

    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    t = tf.transform.translation
    bx = t.x + lx * math.cos(yaw) - ly * math.sin(yaw)
    by = t.y + lx * math.sin(yaw) + ly * math.cos(yaw)
    bearing = math.degrees(math.atan2(by, bx))

    print(f"TF base_link<-laser_frame：平移 ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})、yaw {math.degrees(yaw):+.1f}°")
    print()
    print(f"障礙物在 base_link：x = {bx:+.3f} m、y = {by:+.3f} m、方位角 {bearing:+.1f}°")
    print()

    ok_front = bx > 0
    ok_lateral = abs(by) < 0.15
    ok_bearing = abs(bearing) < 12.0
    def v(b):
        return "PASS" if b else "FAIL"
    print(f"[{v(ok_front)}] 在車頭前方（x>0）")
    print(f"[{v(ok_lateral)}] 橫向偏移小（|y|<0.15 m）")
    print(f"[{v(ok_bearing)}] 方位角接近正前（|角|<12°）")

    allok = ok_front and ok_lateral and ok_bearing
    print()
    print("→ laser_yaw 設定正確" if allok else "→ laser_yaw 仍有問題，別開始建圖")

    rclpy.shutdown()
    return 0 if allok else 1


if __name__ == "__main__":
    # sys.exit 必須包住（Codex F5）：原本裸呼叫 main() 會把 return 1 丟掉、
    # shell 永遠拿到 0——驗證失敗看起來像成功，這在自動化腳本裡是毒。
    sys.exit(main())
