#!/usr/bin/env python3
"""V2V 狀態轉播（leader 車用）——V0 骨架。

把本車融合後的位姿以固定頻率轉播給另一台車：
    訂閱  odometry/filtered  (nav_msgs/Odometry, EKF 輸出, ~30Hz, reliable)
    發布  v2v/state          (nav_msgs/Odometry, 10Hz, best-effort, keep-last-1)

設計依據：docs/design/v2v_architecture.md §4（D6）、§5（D4/D9）。

為什麼發布端是 best-effort + keep-last-1（不是預設的 reliable）：
    reliable 在有損 WiFi 上會重傳，塞住 publisher 並讓 follower 吃到過期資料。
    對「跟隨控制」而言，一筆遲到的位姿比丟掉它更糟——寧可丟，永遠用最新的。

**header.stamp 蓋的是「本節點發送當下的時刻」，不是 EKF 的量測時刻。**
    這是刻意的：follower 端算 (收到時刻 − header.stamp) 就是 V2V 鏈路延遲；
    固定 10Hz 發送下，相鄰兩筆 stamp 間距 ≈100ms，缺一筆就跳成 ≈200ms → 丟包可數。
    兩車時鐘需先用 chrony 對齊（V1 工作包），否則延遲數字只有相對意義。

⚠ v2v/state 的 frame_id 沿用來源（odom / base_link），但那是 **leader 車的** odom。
   兩車 frame 同名（方案 B：TF 樹整棵 remap 進 namespace，車內 frame 名不動），
   所以 follower **絕對不可**拿這個 frame_id 去做 TF lookup——會查到自己的 odom。
   follower 應把收到的位姿當「leader 自身 odom 系下的座標」，用 GAP0 常數平移轉進自身座標。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry

from stm32_bridge.follow_math import V2V_SEQ_COV_INDEX


class V2VRelay(Node):
    def __init__(self):
        super().__init__('v2v_relay')

        self.declare_parameter('rate', 10.0)        # 轉播頻率 (Hz)
        self.declare_parameter('source_timeout', 0.5)   # 多久沒收到來源就停止轉播 (s)
        rate = float(self.get_parameter('rate').value)
        self.source_timeout = float(self.get_parameter('source_timeout').value)

        # 發布端：best-effort + keep-last-1（見檔頭）
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Odometry, 'v2v/state', pub_qos)

        # 訂閱端：車內 topic，維持預設 reliable（同一台機器、不過 WiFi）
        self.sub = self.create_subscription(
            Odometry, 'odometry/filtered', self.odom_cb, 10)

        self.latest = None
        self.last_rx_time = None
        self.sent = 0
        self.warned_stale = False

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'v2v_relay 啟動：odometry/filtered -> v2v/state @{rate:g}Hz '
            f'(best-effort, keep-last-1)，來源逾時 {self.source_timeout:g}s 即停播')

    def odom_cb(self, msg: Odometry):
        self.latest = msg
        self.last_rx_time = self.get_clock().now()
        if self.warned_stale:
            self.get_logger().info('來源 odometry/filtered 已恢復，繼續轉播')
            self.warned_stale = False

    def tick(self):
        if self.latest is None or self.last_rx_time is None:
            return

        # 來源斷了就停播，讓 follower 的 0.5s 逾時保護能如期觸發停車
        age = (self.get_clock().now() - self.last_rx_time).nanoseconds * 1e-9
        if age > self.source_timeout:
            if not self.warned_stale:
                self.get_logger().warn(
                    f'odometry/filtered 已 {age:.1f}s 沒有新資料，停止轉播 v2v/state')
                self.warned_stale = True
            return

        msg = self.latest
        msg.header.stamp = self.get_clock().now().to_msg()   # 蓋「發送時刻」，見檔頭
        # D9：遞增序號塞進 twist.covariance（載體與索引見 follow_math.V2V_SEQ_COV_INDEX）。
        # follower 端算 (收到−header.stamp)=延遲、序號跳號=丟包。序號從 0 連續遞增。
        # ⚠ rclpy 對 float64[36] 欄位「就地改單一元素」不會寫進發出的訊息（getter 回傳副本）——
        #    必須整條陣列重新賦值。2026-07-12 實測踩到：seq 永遠 0，丟包量測全失效。
        cov = list(msg.twist.covariance)
        cov[V2V_SEQ_COV_INDEX] = float(self.sent)
        msg.twist.covariance = cov
        self.pub.publish(msg)

        self.sent += 1
        if self.sent % 100 == 0:   # 10Hz -> 每 10 秒一行
            p = msg.pose.pose.position
            self.get_logger().info(
                f'v2v/state #{self.sent}  x={p.x:+.2f} y={p.y:+.2f} m  '
                f'vx={msg.twist.twist.linear.x:+.2f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = V2VRelay()
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
