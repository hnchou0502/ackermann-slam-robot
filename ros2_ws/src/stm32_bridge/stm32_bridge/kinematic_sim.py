#!/usr/bin/env python3
"""輕量運動學模擬車（V2）——不需要任何硬體，介面與實車 bridge+EKF 完全相同。

    訂閱  cmd_vel            (geometry_msgs/Twist)
    發布  odometry/filtered  (nav_msgs/Odometry)   ← 實車由 EKF 發，這裡由 sim 發
          TF odom -> base_link

設計依據：docs/design/v2v_architecture.md §6（D8）——「輕量運動學 sim 優先，Gazebo 降選配」。
每車起一個實例並各自帶 namespace（/car1、/car2），follower 一行都不用改就能在 sim 與實車間切換。
這是 2026/12/31 kill switch 的保命包：真車來不及時，同一個 launch 換掉 sim 節點即可出 demo 影片。

**忠實復刻實車的三個約束**（不然 sim 跑得通、上實車就撞牆）：
  1. 阿克曼幾何：轉向角 delta = atan(L*w/v)，飽和 ±max_steer_deg（實車靠機構連桿硬限）
     -> 最小轉彎半徑 R_min = L / tan(max_steer) = 0.235/tan30° ≈ 0.407 m。
     指令要求的曲率超過這個，sim 會像實車一樣「轉不過去」，而不是原地畫出不可能的軌跡。
  2. 車不能原地旋轉：|v| < min_turn_speed 時 w 強制歸零（與 bridge_node.cmd_cb 同邏輯）。
  3. dead-man：cmd_timeout 內沒收到 cmd_vel 就停車（與 bridge_node.cmd_timer_tick 同邏輯）。

模型：後軸中心的自行車模型（bicycle model）
    x'     = v cos(theta)
    y'     = v sin(theta)
    theta' = v tan(delta) / L
與韌體 main.c 用的等效角 delta=atan(L*omega/v) 是同一套（見專案 CLAUDE.md「韌體 main.c 設計細節」）。

⚠ sim 沒有里程漂移、沒有輪胎側滑、沒有 IMU 雜訊——它驗證的是**控制邏輯**（follower 會不會切內線、
   會不會撞上去），不是定位精度。定位誤差的實測留給 V3（實車 bag 回放）與 V4。
"""
import math

from stm32_bridge.follow_math import apply_ackermann, min_turn_radius, normalize_angle

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class KinematicSim(Node):
    def __init__(self):
        super().__init__('kinematic_sim')

        self.declare_parameter('wheelbase', 0.235)        # 軸距 L (m)，實車捲尺實測
        self.declare_parameter('max_linear', 0.5)         # 與 bridge_node 相同的夾限
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('max_steer_deg', 30.0)     # 實車機構上限
        self.declare_parameter('min_turn_speed', 0.01)    # 低於此速度不准轉（不能原地旋轉）
        self.declare_parameter('cmd_timeout', 0.5)        # dead-man
        self.declare_parameter('rate', 30.0)              # 積分/發布頻率 (Hz)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.L = float(self.get_parameter('wheelbase').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.max_steer = math.radians(float(self.get_parameter('max_steer_deg').value))
        self.min_turn_speed = float(self.get_parameter('min_turn_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        rate = float(self.get_parameter('rate').value)

        # 位姿一律從原點起算——與實車一致（EKF 從 0 起算航向，見 config/ekf.yaml）。
        # 兩車之間的初始間距 GAP0 由 follower 以常數平移處理，不在 sim 裡設。
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.step)

        r_min = min_turn_radius(self.L, self.max_steer)
        self.get_logger().info(
            f'kinematic_sim 啟動：L={self.L} m，轉向角上限 ±{math.degrees(self.max_steer):.0f}°'
            f' -> 最小轉彎半徑 {r_min:.3f} m；積分 {rate:g}Hz')

    def cmd_cb(self, msg: Twist):
        self.cmd_v = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        self.cmd_w = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:   # 第一筆或異常間隔，不積分這一步
            dt = 0.0

        # dead-man：太久沒收到指令就停車（與實車 bridge 相同語義）
        cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if cmd_age > self.cmd_timeout:
            self.cmd_v = 0.0
            self.cmd_w = 0.0

        v, w = apply_ackermann(self.cmd_v, self.cmd_w, self.L,
                               self.max_steer, self.min_turn_speed)

        # 自行車模型積分（中點法可再精一點，但 30Hz 下歐拉誤差遠小於實車不確定性）
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th = normalize_angle(self.th + w * dt)

        stamp = now.to_msg()
        q = yaw_to_quaternion(self.th)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w      # 注意：發的是**飽和後**的實際角速度
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
