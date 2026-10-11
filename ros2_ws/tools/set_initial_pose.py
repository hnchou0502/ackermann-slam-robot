#!/usr/bin/env python3
"""餵給 AMCL 初始位置（等同 RViz 的 2D Pose Estimate）。
用法：python3 set_initial_pose.py <x> <y> <yaw_rad>
"""
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

X = float(sys.argv[1])
Y = float(sys.argv[2])
YAW = float(sys.argv[3])


def main():
    rclpy.init()
    node = Node("set_initial_pose")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = X
    msg.pose.pose.position.y = Y
    msg.pose.pose.orientation.z = math.sin(YAW / 2.0)
    msg.pose.pose.orientation.w = math.cos(YAW / 2.0)
    # AMCL 預設初始不確定度：位置 σ²=0.25(±0.5m)、航向 σ²=0.0685(±15°)
    cov = [0.0] * 36
    cov[0] = 0.25
    cov[7] = 0.25
    cov[35] = 0.06853
    msg.pose.covariance = cov

    # 多發幾次，確保 AMCL 收到（它可能還在啟動）
    for i in range(8):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.4)

    print(f"已送出初始位置：x={X:+.3f} y={Y:+.3f} yaw={math.degrees(YAW):+.1f}°")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
