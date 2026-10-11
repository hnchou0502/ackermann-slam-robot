#!/usr/bin/env python3
"""送一個導航目標給 Nav2，並即時回報進度／結果。
用法：python3 send_goal.py <x> <y> <yaw_rad> [逾時秒]
"""
import sys
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

X = float(sys.argv[1])
Y = float(sys.argv[2])
YAW = float(sys.argv[3])
TIMEOUT = float(sys.argv[4]) if len(sys.argv) > 4 else 90.0


def main():
    rclpy.init()
    node = Node("send_goal")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    if not client.wait_for_server(timeout_sec=15.0):
        print("連不上 navigate_to_pose action server")
        rclpy.shutdown()
        return 1

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = X
    goal.pose.pose.position.y = Y
    goal.pose.pose.orientation.z = math.sin(YAW / 2.0)
    goal.pose.pose.orientation.w = math.cos(YAW / 2.0)

    print(f"送出目標：({X:+.3f}, {Y:+.3f}) 朝向 {math.degrees(YAW):+.0f}°")

    state = {"last": 0.0}

    def on_feedback(fb):
        f = fb.feedback
        now = time.time()
        if now - state["last"] < 2.0:
            return
        state["last"] = now
        p = f.current_pose.pose.position
        print(f"  進行中：位置 ({p.x:+.3f}, {p.y:+.3f}) "
              f"剩餘 {f.distance_remaining:.2f} m 已走 {f.number_of_recoveries} 次復原")

    send = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("❌ 目標被拒絕（通常＝規劃不出路徑）")
        rclpy.shutdown()
        return 1
    print("目標已接受，開始導航……")

    result_fut = handle.get_result_async()
    t0 = time.time()
    while rclpy.ok() and not result_fut.done():
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - t0 > TIMEOUT:
            print(f"⏱ 超過 {TIMEOUT:.0f}s 未完成，取消目標")
            handle.cancel_goal_async()
            rclpy.spin_once(node, timeout_sec=2.0)
            rclpy.shutdown()
            return 1

    res = result_fut.result()
    code = res.status
    names = {2: "取消中", 3: "執行中", 4: "成功", 5: "被取消", 6: "失敗"}
    print(f"\n結果：{names.get(code, code)}（status={code}）耗時 {time.time()-t0:.1f}s")
    rclpy.shutdown()
    return 0 if code == 4 else 1


if __name__ == "__main__":
    sys.exit(main())
