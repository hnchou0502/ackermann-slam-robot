#!/usr/bin/env python3
"""監看 Nav2：使用者在 Foxglove/RViz 點目標後，回報收到什麼、規劃結果、走到哪。
用法：python3 nav_monitor.py [秒數]
"""
import sys
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0

STATUS = {0: "未知", 1: "已接受", 2: "執行中", 3: "取消中",
          4: "成功 ✅", 5: "已取消", 6: "失敗 ❌"}


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


class M(Node):
    def __init__(self):
        super().__init__("nav_monitor")
        self.last_status = None
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                 self.on_status, 10)
        print("監看中……請在 Foxglove 點一個目標")

    def on_goal(self, msg):
        p = msg.pose.position
        print(f"\n[{time.strftime('%H:%M:%S')}] 收到目標："
              f"({p.x:+.3f}, {p.y:+.3f}) 朝向 {yaw_of(msg.pose.orientation):+.0f}°"
              f"  frame={msg.header.frame_id}")
        if msg.header.frame_id != "map":
            print(f"   ⚠ frame_id 不是 map 而是 '{msg.header.frame_id}'"
                  f" —— Foxglove 的顯示座標系要設成 map，否則目標位置會錯")

    def on_plan(self, msg):
        n = len(msg.poses)
        if n == 0:
            print("   ❌ 規劃出空路徑（規劃失敗）")
            return
        length = 0.0
        for a, b in zip(msg.poses, msg.poses[1:]):
            length += math.hypot(b.pose.position.x - a.pose.position.x,
                                 b.pose.position.y - a.pose.position.y)
        print(f"   ✅ 規劃成功：{n} 個路徑點、全長 {length:.2f} m")

    def on_status(self, msg):
        if not msg.status_list:
            return
        s = msg.status_list[-1].status
        if s != self.last_status:
            self.last_status = s
            print(f"   狀態 → {STATUS.get(s, s)}")


def main():
    rclpy.init()
    node = M()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
