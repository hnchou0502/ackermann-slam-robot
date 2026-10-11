#!/usr/bin/env python3
"""靜止陀螺儀 bias/σ 量測，對照 CLAUDE.md 驗收標準。
判準（2026-07-10 十分鐘量測定的）：|bias| < 0.2°/s、σ < 0.3°/s、峰值 < 0.8°/s。
用法：python3 gyro_bias.py [秒數]
"""
import sys
import math
import statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0


class G(Node):
    def __init__(self):
        super().__init__("gyro_bias")
        self.vals = []
        self.create_subscription(Imu, "/imu/data", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.vals.append(msg.angular_velocity.z)


def main():
    rclpy.init()
    node = G()
    t0 = node.get_clock().now()
    while (node.get_clock().now() - t0).nanoseconds / 1e9 < DUR:
        rclpy.spin_once(node, timeout_sec=0.1)

    v = [math.degrees(x) for x in node.vals]
    if len(v) < 10:
        print(f"樣本太少（{len(v)} 筆），/imu/data 可能沒在發")
        rclpy.shutdown()
        return 1

    bias = statistics.fmean(v)
    sigma = statistics.pstdev(v)
    peak = max(abs(x) for x in v)
    drift = bias * 60  # 每分鐘航向漂移

    def verdict(ok):
        return "OK" if ok else "超標"

    print(f"樣本 {len(v)} 筆 / {DUR:.0f}s")
    print(f"bias  {bias:+.4f} °/s   判準 |bias|<0.2   → {verdict(abs(bias) < 0.2)}")
    print(f"sigma  {sigma:.4f} °/s   判準 σ<0.3        → {verdict(sigma < 0.3)}")
    print(f"peak   {peak:.4f} °/s   判準 peak<0.8     → {verdict(peak < 0.8)}")
    print(f"→ 換算航向漂移約 {drift:+.2f} °/分鐘")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
