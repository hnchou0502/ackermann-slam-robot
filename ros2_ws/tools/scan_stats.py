#!/usr/bin/env python3
"""量測 /scan 品質：頻率、每圈點數、有效點比例、量程分布。
用法：python3 scan_stats.py [秒數]
"""
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0


class Stats(Node):
    def __init__(self):
        super().__init__("scan_stats")
        self.msgs = []
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.msgs.append(msg)


def main():
    rclpy.init()
    node = Stats()
    t0 = node.get_clock().now()
    while (node.get_clock().now() - t0).nanoseconds / 1e9 < DUR:
        rclpy.spin_once(node, timeout_sec=0.1)

    m = node.msgs
    if not m:
        print("沒收到任何 /scan —— 驅動沒發或 QoS 不合")
        rclpy.shutdown()
        return 1

    # 用 header.stamp 算頻率（避開「收到時刻」的誤差）
    t_first = m[0].header.stamp.sec + m[0].header.stamp.nanosec / 1e9
    t_last = m[-1].header.stamp.sec + m[-1].header.stamp.nanosec / 1e9
    span = t_last - t_first
    hz = (len(m) - 1) / span if span > 0 else float("nan")

    s = m[len(m) // 2]  # 取中間一筆當代表
    n = len(s.ranges)
    valid = [r for r in s.ranges
             if not math.isinf(r) and not math.isnan(r) and s.range_min <= r <= s.range_max]
    ratio = len(valid) / n * 100 if n else 0

    print(f"收到 {len(m)} 筆，時間跨度 {span:.2f}s")
    print(f"頻率        {hz:.2f} Hz")
    print(f"每圈點數    {n}")
    print(f"有效點      {len(valid)} / {n} = {ratio:.1f}%")
    print(f"量程設定    range_min={s.range_min:.3f} m  range_max={s.range_max:.1f} m")
    if valid:
        vs = sorted(valid)
        print(f"實測距離    最近 {vs[0]:.3f} m / 中位 {vs[len(vs)//2]:.3f} m / 最遠 {vs[-1]:.3f} m")
        far = [r for r in valid if r > 6.0]
        print(f"超過 6m 的點  {len(far)} 個（X4 打不到這麼遠，這是 C1 的加分項）")
    print(f"frame_id    {s.header.frame_id}")
    print(f"角度範圍    {math.degrees(s.angle_min):.1f}° ~ {math.degrees(s.angle_max):.1f}°"
          f"  解析度 {math.degrees(s.angle_increment):.3f}°/點")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
