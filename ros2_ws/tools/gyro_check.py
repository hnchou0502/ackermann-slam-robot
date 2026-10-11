#!/usr/bin/env python3
"""量陀螺零漂：車靜止時感測器回報的假旋轉速度。判準 <0.2 度/秒。

建圖／自主探索前必跑。零漂大時地圖會歪掉且**完全不報錯**（專案地雷 2026-07-13）。
超標的修法＝車完全靜止後按 STM32 板上 RESET，然後 3 秒內別碰它（含別踩地板）。

實測參考（car1）：搬車後未重校 -0.0998；重校後 +0.0171 ~ +0.0084 度/秒。
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
LIMIT_DEG_S = 0.2


class G(Node):
    def __init__(self):
        super().__init__('gyro_check')
        self.v = []
        self.create_subscription(Imu, '/imu/data',
                                 lambda m: self.v.append(m.angular_velocity.z), 50)


def main():
    rclpy.init()
    g = G()
    t0 = g.get_clock().now().nanoseconds
    while (g.get_clock().now().nanoseconds - t0) < DURATION * 1e9:
        rclpy.spin_once(g, timeout_sec=0.2)
    n = len(g.v)
    if not n:
        print('沒收到 /imu/data（bringup 有在跑嗎？）')
        rclpy.shutdown()
        return 1
    m = sum(g.v) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in g.v) / n)
    print('n=%d  bias=%.5f rad/s = %+.4f deg/s  sigma=%.4f deg/s  峰值=%+.4f deg/s'
          % (n, m, m * 57.2958, sd * 57.2958, max(g.v, key=abs) * 57.2958))
    passed = abs(m * 57.2958) < LIMIT_DEG_S
    print('判準 <%.1f deg/s -> %s' % (LIMIT_DEG_S,
          'PASS' if passed else 'FAIL（按 STM32 RESET 靜置 3 秒重校）'))
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
