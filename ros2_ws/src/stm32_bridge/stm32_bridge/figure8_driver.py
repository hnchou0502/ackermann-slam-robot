#!/usr/bin/env python3
"""開迴路 8 字驅動（V2 驗收用）——固定速度 + 正弦角速度，發 cmd_vel。

    發布  cmd_vel  (geometry_msgs/Twist)

為什麼不用 teleop 人工遙控：V2 的驗收條件是「follower 間距誤差 RMS < 0.3m、無碰撞」，
人手遙控每次都不一樣，數字不可重現、迴歸測不了。這支腳本讓 leader 的軌跡是**確定性**的，
同一組參數跑兩次結果一致 —— 驗收才有意義。實車 demo（V4）仍然是人遙控。

參數（預設值已檢查落在阿克曼可行域內，見 follow_math.figure8_params 的說明）：
    speed          0.35 m/s   固定前進速度
    turn_radius    0.9  m     8 字的最小瞬時轉彎半徑（必須 > R_min=0.407m）
    warmup         3.0  s     起跑前先直線走一段，讓 follower 攢到麵包屑
    cycles         1.0        跑幾個 8 字週期（一個週期 ≈ 50.8s）

跑完會自動送停車並結束 process（exit 0），方便腳本串接。
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from stm32_bridge.follow_math import (
    apply_ackermann,
    figure8_omega,
    figure8_params,
    min_turn_radius,
)


class Figure8Driver(Node):
    def __init__(self):
        super().__init__('figure8_driver')

        self.declare_parameter('speed', 0.35)
        self.declare_parameter('turn_radius', 0.9)
        self.declare_parameter('warmup', 3.0)
        self.declare_parameter('cycles', 1.0)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('wheelbase', 0.235)
        self.declare_parameter('max_steer_deg', 30.0)

        self.speed = float(self.get_parameter('speed').value)
        radius = float(self.get_parameter('turn_radius').value)
        self.warmup = float(self.get_parameter('warmup').value)
        cycles = float(self.get_parameter('cycles').value)
        rate = float(self.get_parameter('rate').value)
        self.L = float(self.get_parameter('wheelbase').value)
        self.max_steer = math.radians(float(self.get_parameter('max_steer_deg').value))

        r_min = min_turn_radius(self.L, self.max_steer)
        if radius <= r_min:
            raise ValueError(
                f'turn_radius={radius} m 小於阿克曼最小轉彎半徑 {r_min:.3f} m，車轉不過去')

        self.amp, self.period = figure8_params(self.speed, radius)
        self.duration = self.warmup + cycles * self.period
        self.done = False

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'8 字驅動：v={self.speed:g} m/s、最小半徑={radius:g} m '
            f'-> 角速度振幅 {self.amp:.4f} rad/s、週期 {self.period:.1f} s；'
            f'暖機 {self.warmup:g} s、共跑 {self.duration:.1f} s')

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        if t >= self.duration:
            if not self.done:
                self.pub.publish(Twist())   # 停車
                self.get_logger().info(f'8 字跑完（{t:.1f} s），已送停車')
                self.done = True
            return

        msg = Twist()
        if t < self.warmup:
            # 暖機：直線前進，讓 follower 攢到足夠的麵包屑才開始跟
            msg.linear.x = self.speed
            msg.angular.z = 0.0
        else:
            w = figure8_omega(t - self.warmup, self.amp, self.period)
            # 自己先夾一次，確保發出去的就是實車做得到的指令
            v, w = apply_ackermann(self.speed, w, self.L, self.max_steer)
            msg.linear.x = v
            msg.angular.z = w
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Driver()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # 收尾安全：連送停車，避免留下殘餘指令（實車上這條很重要）
        for _ in range(5):
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
