"""持續記錄 /odom 輪速，直到資料中斷或逾時；印出時間序列與「歸零時刻」。

EX3 用來量真正的「輪速歸零時間」——先前只量行程消失時間，那不等於車停時間
（bridge 在關閉流程一開始就會送停車指令，不是等行程消失才停）。

用法：python3 monitor_speed.py <總秒數> [歸零門檻]
輸出每筆 (相對時間, 輪速)，最後印第一次連續 3 筆低於門檻的時刻。
"""
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
ZERO = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02


def main():
    rclpy.init()
    n = Node('monitor_speed')
    rows = []
    t0 = n.get_clock().now().nanoseconds

    def cb(m):
        t = (n.get_clock().now().nanoseconds - t0) / 1e9
        rows.append((t, m.twist.twist.linear.x))

    n.create_subscription(Odometry, '/odom', cb, 50)
    while (n.get_clock().now().nanoseconds - t0) < DURATION * 1e9:
        rclpy.spin_once(n, timeout_sec=0.05)

    if not rows:
        print('MONITOR: 完全沒收到 /odom')
        rclpy.shutdown()
        return

    print('MONITOR: 共 %d 筆，涵蓋 %.2f ~ %.2f 秒' % (len(rows), rows[0][0], rows[-1][0]))
    print('  最大輪速 = %.3f m/s' % max(abs(v) for _, v in rows))

    # 第一次「連續 3 筆都低於門檻」的時刻
    zero_at = None
    for i in range(len(rows) - 2):
        if all(abs(rows[i + k][1]) < ZERO for k in range(3)):
            zero_at = rows[i][0]
            break
    if zero_at is not None:
        print('  輪速歸零(<%.2f m/s, 連續3筆)於 t=%.2f 秒' % (ZERO, zero_at))
    else:
        print('  在資料涵蓋範圍內沒有歸零')
    print('  最後 8 筆 (t, v)：',
          ' '.join('(%.2f,%.3f)' % r for r in rows[-8:]))
    print('  資料最後時刻 t=%.2f（之後 /odom 中斷＝bridge 已死）' % rows[-1][0])
    rclpy.shutdown()


main()
