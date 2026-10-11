#!/usr/bin/env python3
"""EX4／RM3 開跑前量「車擺的位置夠不夠開闊」。

**為什麼這件事值得專門量**：離線預演（`explore_dryrun.py`，2026-08-11）發現
自主探索的成敗**對起點極度敏感**——起步時光達可見面積 <3 m² 的多數失敗、
**≥5 m² 達標率 86%**。這是目前已知對 EX4 結果影響最大的單一變因，
但 preflight 只能問人「車有沒有放在開闊處」，人眼判斷不準。這支把它變成數字。

做法：取一幀 `/scan`，把每條射線的端點連成多邊形（鞋帶公式）算面積＝
「車從現在這個位置一眼看得到的地面範圍」。無效／超量程的射線以 range_max 計，
因為那代表「那個方向是空的」，本來就該算進可見範圍。

用法（bringup 要在跑，車不會動）：
    python3 check_start_openness.py [--min-area 5.0]

退出碼：0＝夠開闊；1＝不夠，建議換位置。
"""
import argparse
import math
import sys
import time

GOOD_AREA_M2 = 5.0
"""達標門檻。依據＝explore_dryrun 多起點分布：≥5 m² 達標率 86%。"""

BAD_AREA_M2 = 3.0
"""低於此值多數失敗（同上出處）。"""

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def polygon_area(pts):
    """鞋帶公式。pts＝[(x, y), ...] 依角度排序。"""
    a = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-area", type=float, default=GOOD_AREA_M2)
    ap.add_argument("--timeout", type=float, default=20.0)
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class Grab(Node):
        def __init__(self):
            super().__init__("start_openness")
            self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)
            self.msg = None

        def cb(self, m):
            if self.msg is None:
                self.msg = m

    rclpy.init()
    try:
        n = Grab()
        t0 = time.time()
        while rclpy.ok() and n.msg is None and time.time() - t0 < a.timeout:
            rclpy.spin_once(n, timeout_sec=0.5)
        m = n.msg
    finally:
        rclpy.shutdown()

    if m is None:
        print("❌ %.0f 秒內收不到 /scan——bringup 沒在跑？" % a.timeout)
        return 1

    pts = []
    valid = 0
    for i, r in enumerate(m.ranges):
        ang = m.angle_min + i * m.angle_increment
        if math.isfinite(r) and m.range_min <= r <= m.range_max:
            valid += 1
        else:
            # 無效／超量程＝那個方向是空的，以最大量程計入可見範圍
            r = m.range_max
        pts.append((r * math.cos(ang), r * math.sin(ang)))

    area = polygon_area(pts)
    nearest = min((r for r in m.ranges
                   if math.isfinite(r) and m.range_min <= r <= m.range_max), default=float("inf"))

    print("光達點數 %d（有效 %d, %.1f%%）  最近障礙 %.2f m" %
          (len(m.ranges), valid, 100.0 * valid / max(1, len(m.ranges)), nearest))
    print("起步可見面積：**%.2f m²**" % area)
    print("判準（來自 explore_dryrun 多起點分布）：≥%.1f m² 達標率 86%%；<%.1f m² 多數失敗"
          % (GOOD_AREA_M2, BAD_AREA_M2))

    if area >= a.min_area:
        print("✅ 夠開闊，可以開跑")
        return 0
    if area < BAD_AREA_M2:
        print("❌ 太封閉（<%.1f m²）——換個開闊的位置再跑，硬跑多半白費一趟" % BAD_AREA_M2)
    else:
        print("⚠ 介於中間（%.1f~%.1f m²）——建議往開闊處挪，或知情接受較低的達標機率"
              % (BAD_AREA_M2, GOOD_AREA_M2))
    return 1


if __name__ == "__main__":
    sys.exit(main())
