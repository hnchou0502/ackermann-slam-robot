#!/usr/bin/env python3
"""麵包屑軌跡跟隨 + 純追蹤（V2 的 follower 車）。

    訂閱  v2v/state          (nav_msgs/Odometry, best-effort)  leader 位姿（leader 自己的 odom 系）
          odometry/filtered  (nav_msgs/Odometry)               自身位姿（實車來自 EKF、模擬來自 kinematic_sim）
    發布  cmd_vel            (geometry_msgs/Twist)

設計依據：docs/design/v2v_architecture.md §4（D5/D6）。

**為什麼跟「軌跡」而不是直接追 leader 的當下位置**：直接追會切內線——轉彎時 follower 抄近路，
會往 leader 的內側撞上去。麵包屑讓 follower 走 leader 走過的路（網查坐實為標準做法）。

**座標系**（方案 B：兩車 frame 同名、各自 odom 原點在各自起點）
    v2v/state 裡的位姿是 **leader 自己 odom 系**下的座標，不能直接用，也不能拿 frame_id 去做 TF lookup。
    座標對齊（D5 校正起點法）：leader 的 odom 從自身原點起（sim 從 (0,0,0)、實車 EKF 的
    odom 也歸零起步），故以 leader odom 原點為**固定基準**，把 leader 位置放到 follower 正前方
    GAP0（實作 align_leader_point；origin=(0,0,0) 時等價單純 +GAP0 平移）。
    這個基準是絕對的、follower 與評分器共用同一個。
    ⚠ **不可改用「本節點收到的第一筆位姿」當基準**——follower 與評分器啟動時刻不同，收到的
       第一筆會差好幾秒運動量，兩邊座標系就對不上（2026-07-11 踩過，V2 評分整個爆掉）。
    初始航向對不齊仍是主導誤差（±3° 會讓整條軌跡旋轉），實車靠對齊治具壓到 <1°；模擬裡沒有。

**速度控制＝leader 速度前饋 ＋ 間距 P 回授**
    設計文件只寫了「沿軌跡間距 P 控制」，但純 P 有穩態誤差：間距一旦回到 GAP_TARGET，
    速度就變 0，follower 隨即掉隊，最後穩定在 gap = GAP_TARGET + v_leader/Kp
    （Kp=0.8、v_leader=0.35 時是 1.64m，誤差 0.44m —— 直接超過 RMS<0.3m 的驗收線）。
    加上 leader 速度前饋後，穩態時 v = v_leader 且 gap = GAP_TARGET，穩態誤差為零。

**安全**（四層，與實車 bridge 的保護疊加）
    1. leader 資料逾時 leader_timeout -> 立刻發停車
    2. 沿軌跡間距 < gap_min -> 速度歸零（不倒車，v 恆 >= 0）
    3. 指令送出前過一次阿克曼可行域（低速不轉、轉向角飽和）
    4. ⚠ follower 會**持續**發 cmd_vel，這會餵飽 bridge 的 dead-man ——
       實車 demo 必須另外預留急停手段（V4 runbook：預開一個 ssh 終端 `pkill -f follower`）。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from stm32_bridge.follow_math import (
    BreadcrumbTrail,
    LinkMonitor,
    V2V_SEQ_COV_INDEX,
    adaptive_lookahead,
    align_leader_point,
    apply_ackermann,
    pure_pursuit_curvature,
    world_to_base,
    yaw_from_quaternion,
)


class Follower(Node):
    def __init__(self):
        super().__init__('follower')

        self.declare_parameter('gap0', 1.0)            # 開場時 follower 落後 leader 的距離 (m)
        self.declare_parameter('gap_target', 1.2)      # 想維持的跟車距離 (m)
        self.declare_parameter('gap_min', 0.5)         # 低於此距離就停 (m)
        self.declare_parameter('lookahead', 0.8)       # 純追蹤前視距離上限 (m)
        self.declare_parameter('lookahead_gamma', 2.3)  # 前視隨速縮放 ld=γ·v（V3 實測低速切彎的修正）
        self.declare_parameter('lookahead_min', 0.35)   # 前視下限，防低速轉向震盪 (m)
        self.declare_parameter('kp_gap', 0.8)          # 間距誤差 -> 速度的 P 增益
        self.declare_parameter('breadcrumb_spacing', 0.05)
        self.declare_parameter('leader_timeout', 0.5)  # leader 資料逾時 -> 停車 (s)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('wheelbase', 0.235)
        self.declare_parameter('max_steer_deg', 30.0)
        self.declare_parameter('min_turn_speed', 0.01)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('csv_path', '')   # D9：非空則收尾把 V2V 鏈路延遲/丟包逐筆寫此檔

        self.gap0 = float(self.get_parameter('gap0').value)
        self.gap_target = float(self.get_parameter('gap_target').value)
        self.gap_min = float(self.get_parameter('gap_min').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.ld_gamma = float(self.get_parameter('lookahead_gamma').value)
        self.ld_min = float(self.get_parameter('lookahead_min').value)
        self.kp_gap = float(self.get_parameter('kp_gap').value)
        self.leader_timeout = float(self.get_parameter('leader_timeout').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.L = float(self.get_parameter('wheelbase').value)
        self.max_steer = math.radians(float(self.get_parameter('max_steer_deg').value))
        self.min_turn_speed = float(self.get_parameter('min_turn_speed').value)
        rate = float(self.get_parameter('rate').value)

        self.trail = BreadcrumbTrail(
            min_spacing=float(self.get_parameter('breadcrumb_spacing').value))

        self.x = self.y = self.th = 0.0
        self.have_self_pose = False
        self.leader_v = 0.0
        self.last_leader_time = None
        self.stopped_for_timeout = False
        self.ticks = 0

        # D9：V2V 鏈路延遲/丟包量測（純統計在 follow_math.LinkMonitor）
        self.csv_path = str(self.get_parameter('csv_path').value)
        self.link = LinkMonitor()

        # ⚠ 必須用 best-effort 訂閱：v2v_relay 是 best-effort publisher，
        #   reliable 訂閱端與它 QoS 不相容，會一筆都收不到（而且不會報錯）。
        leader_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.leader_sub = self.create_subscription(
            Odometry, 'v2v/state', self.leader_cb, leader_qos)
        self.self_sub = self.create_subscription(
            Odometry, 'odometry/filtered', self.self_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(1.0 / rate, self.control_tick)
        self.get_logger().info(
            f'follower 啟動：GAP0={self.gap0:g} m、目標間距={self.gap_target:g} m、'
            f'最小間距={self.gap_min:g} m、lookahead={self.lookahead:g} m、Kp={self.kp_gap:g}')

    def self_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        self.th = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        self.have_self_pose = True

    def leader_cb(self, msg: Odometry):
        # 基準固定為 leader 的 odom 原點（跨節點一致的絕對基準；見檔頭「座標對齊」）。
        # 不可改用「本節點收到的第一筆」——啟動時刻不同會害 follower/評分器座標系錯位。
        p = msg.pose.pose.position
        fx, fy = align_leader_point(p.x, p.y, (0.0, 0.0, 0.0), self.gap0)
        self.trail.append(fx, fy)
        self.leader_v = msg.twist.twist.linear.x
        now = self.get_clock().now()
        self.last_leader_time = now

        # D9：延遲＝收到時刻 − relay 蓋的發送時刻（header.stamp）；序號來自 twist.covariance。
        # 兩者同一節點時鐘域相減，實車需 chrony 對齊才有絕對意義（見 LinkMonitor 註）。
        send_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        recv_t = now.nanoseconds * 1e-9
        seq = msg.twist.covariance[V2V_SEQ_COV_INDEX]
        self.link.record(send_t, recv_t, seq)

        if self.stopped_for_timeout:
            self.get_logger().info('leader 資料恢復，繼續跟隨')
            self.stopped_for_timeout = False

    def stop(self):
        self.cmd_pub.publish(Twist())

    def control_tick(self):
        if not self.have_self_pose or self.last_leader_time is None:
            self.stop()
            return

        # 安全 1：leader 斷線就停
        age = (self.get_clock().now() - self.last_leader_time).nanoseconds * 1e-9
        if age > self.leader_timeout:
            if not self.stopped_for_timeout:
                self.get_logger().warn(f'leader 資料已 {age:.1f}s 未更新，停車')
                self.stopped_for_timeout = True
            self.stop()
            return

        if len(self.trail) < 2:
            self.stop()
            return

        gap = self.trail.path_gap(self.x, self.y)

        # 安全 2：貼太近就停（v 恆 >= 0，follower 永不倒車）
        if gap < self.gap_min:
            self.stop()
            self._log(gap, 0.0, 0.0)
            return

        # 速度：leader 速度前饋 + 間距誤差 P 回授（見檔頭）
        v = self.leader_v + self.kp_gap * (gap - self.gap_target)
        v = max(0.0, min(self.max_linear, v))

        # 轉向：純追蹤取 lookahead 目標點（前視隨速縮放——低速看太遠會切彎，V3 實測）
        ld = adaptive_lookahead(v, self.ld_gamma, self.ld_min, self.lookahead)
        target = self.trail.lookahead_point(self.x, self.y, ld)
        if target is None:
            self.stop()
            return
        bx, by = world_to_base(target[0], target[1], self.x, self.y, self.th)
        w = v * pure_pursuit_curvature(bx, by)
        w = max(-self.max_angular, min(self.max_angular, w))

        # 安全 3：送出去的必須是實車做得到的指令
        v, w = apply_ackermann(v, w, self.L, self.max_steer, self.min_turn_speed)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

        self.trail.prune_before(self.x, self.y)
        self._log(gap, v, w)

    def _log(self, gap: float, v: float, w: float):
        self.ticks += 1
        if self.ticks % 40 == 0:   # 20Hz -> 每 2 秒一行
            self.get_logger().info(
                f'gap={gap:.2f} m (目標 {self.gap_target:g})  v={v:+.2f} m/s  '
                f'w={w:+.2f} rad/s  麵包屑={len(self.trail)}')

    def write_link_report(self):
        """D9 收尾：印 V2V 鏈路延遲/丟包統計，csv_path 非空時逐筆寫 CSV。"""
        s = self.link.summary()
        if s['n'] == 0:
            self.get_logger().warn(
                'V2V 鏈路量測：一筆都沒收到——檢查 v2v/state 的 topic/remap 有沒有接上。')
            return
        loss_pct = float('nan') if math.isnan(s['loss_rate']) else s['loss_rate'] * 100.0
        print('\n' + '=' * 62)
        print('D9 V2V 鏈路延遲/丟包統計')
        print('=' * 62)
        print(f'收到筆數        : {s["n"]}（序號跨度應收 {s["expected"]}）')
        print(f'丟包            : {s["lost"]} 筆   丟包率 {loss_pct:.1f}%')
        print(f'重複 / 亂序     : {s["duplicated"]} / {s["reordered"]}')
        print(f'延遲 平均 / RMS : {s["delay_mean"]*1000:.1f} / {s["delay_rms"]*1000:.1f} ms')
        print(f'延遲 最小 / 最大: {s["delay_min"]*1000:.1f} / {s["delay_max"]*1000:.1f} ms')
        print('-' * 62)
        print('⚠ 延遲絕對值需兩車 chrony 對時才有意義（sim 同時鐘域會接近 0）；')
        print('  丟包數/序號與時鐘無關，任何情況都可信。')
        if self.csv_path:
            with open(self.csv_path, 'w', encoding='utf-8') as f:
                f.write('seq,t_send,t_recv,delay_ms,gap_lost\n')
                for seq, ts, tr, dms, gap in self.link.rows():
                    f.write(f'{int(seq)},{ts:.6f},{tr:.6f},{dms:.3f},{int(gap)}\n')
            print(f'CSV : {self.csv_path}')
        print('=' * 62)


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):   # 收尾安全：連送停車
            node.stop()
        node.write_link_report()   # D9：印鏈路統計＋寫 CSV
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
