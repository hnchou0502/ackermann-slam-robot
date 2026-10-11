"""讀 /odom 的輪速幾秒，印出最大值（EX3 故障注入用來確認「注入前車確實在動」）。

用 rclpy 直接訂閱，不用 `ros2 topic echo`——後者在這台 Pi 上偶爾在 timeout 內
一筆都收不到（2026-08-11 實測），會讓測試誤以為車沒動。
"""
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0


def main():
    rclpy.init()
    n = Node('read_speed')
    vals = []
    n.create_subscription(Odometry, '/odom',
                          lambda m: vals.append(m.twist.twist.linear.x), 20)
    t0 = n.get_clock().now().nanoseconds
    while (n.get_clock().now().nanoseconds - t0) < DURATION * 1e9:
        rclpy.spin_once(n, timeout_sec=0.1)
    if vals:
        print('SPEED_MAX=%.3f SPEED_LAST=%.3f N=%d'
              % (max(abs(v) for v in vals), vals[-1], len(vals)))
    else:
        print('SPEED_MAX=none N=0')
    rclpy.shutdown()


main()
