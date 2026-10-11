#!/usr/bin/env python3
"""V2 驗收評分器——把「follower 跟得好不好」變成可機器檢查的數字。

    訂閱  <leader_topic>    預設 /car1/odometry/filtered
          <follower_topic>  預設 /car2/odometry/filtered
    輸出  終端摘要 + CSV（每個 follower 取樣一列）

驗收判準（docs/design/v2v_architecture.md §7，V2 那列）：
    1. 沿軌跡間距誤差 RMS < 0.3 m
    2. 無碰撞：兩車中心距全程 > 車長 0.337 m
    3. 不切內線：follower 偏離 leader 路徑的 cross-track error（穩態段）< 0.15 m

第 3 條是設計文件沒寫成數字、但**麵包屑機制真正要解決的問題**（§4 D6：直接追 leader
當下位置會在轉彎時抄近路撞上去）。前兩條都測不到它——切內線時間距仍可維持、也未必碰撞。
門檻取 0.15m ≈ 車寬 0.191m 的 80%：偏離超過大半個車身就談不上「走同一條路」。

⚠ cross-track 只在 settle 秒之後才計分：開場 follower 站在 leader 軌跡**起點後方 GAP0 公尺**，
   還沒踏上那條路徑，此時「偏離路徑的距離」本來就等於 GAP0，不是控制誤差。

**這支腳本自己重建 leader 的麵包屑並獨立算間距**，不讀 follower 內部狀態——
不能拿被評者自己算的數字當作它表現good的證據。

座標系：以 leader odom 原點為固定基準，把 leader 軌跡放到「follower 正前方 gap0、同向」
（與 follower 節點共用 align_leader_point、共用同一個絕對基準，兩邊必須一致）。
⚠ 不可用「本節點收到的第一筆」當基準——評分器與 follower 啟動時刻不同，第一筆差好幾秒
   運動量，座標系會錯位，評分整個作廢（2026-07-11 踩過）。

時間對齊：leader 與 follower 的取樣不同步，對每個 follower 樣本取「時間最近的」leader 樣本
（30Hz 下最差 ~17ms，車速 0.35m/s -> 位置誤差 ~6mm，遠小於判準尺度）。

暖機：leader 靜止時 follower 本來就不該動，這段不代表跟隨性能。摘要會**同時**報
「全程 RMS」與「leader 開動後 RMS」，不藏任何一個；PASS/FAIL 以較嚴格的全程 RMS 為準。
"""
import argparse
import bisect
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

from stm32_bridge.follow_math import (
    BreadcrumbTrail,
    align_leader_point,
    score_following,
)


class ScoreFollow(Node):
    def __init__(self, args):
        super().__init__('score_follow')
        self.args = args

        self.trail = BreadcrumbTrail(min_spacing=0.02, max_points=100000)
        self.leader_samples = []      # (t, x, y, v)  x/y 已對齊進 follower 座標系
        self.rows = []                # (t, fx, fy, lx, ly, gap, err, center_dist)
        self.t0 = None
        self.leader_moving_since = None

        # BEST_EFFORT 訂閱者對 reliable 與 best-effort 發布者都相容；用整數 depth 會退成
        # RELIABLE，一旦 --leader-topic 指到 best-effort 的 /carX/v2v/state 就一筆都收不到
        # 且不報錯（與 follower.py 記過的同一個雷）。這裡一律用 best-effort 避開。
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Odometry, args.leader_topic, self.leader_cb, qos)
        self.create_subscription(Odometry, args.follower_topic, self.follower_cb, qos)
        self.get_logger().info(
            f'評分中：leader={args.leader_topic}  follower={args.follower_topic}  '
            f'gap0={args.gap0:g}  目標間距={args.gap_target:g}  '
            f'碰撞門檻={args.collision_dist:g} m  時長={args.duration:g}s')

    def _stamp(self, msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def leader_cb(self, msg: Odometry):
        t = self._stamp(msg)
        if self.t0 is None:
            self.t0 = t
        p = msg.pose.pose.position
        x, y = align_leader_point(p.x, p.y, (0.0, 0.0, 0.0), self.args.gap0)
        v = msg.twist.twist.linear.x
        self.trail.append(x, y)
        self.leader_samples.append((t, x, y, v))
        if self.leader_moving_since is None and abs(v) > 0.05:
            self.leader_moving_since = t

    def follower_cb(self, msg: Odometry):
        if not self.leader_samples or len(self.trail) < 2:
            return
        t = self._stamp(msg)
        fx = msg.pose.pose.position.x
        fy = msg.pose.pose.position.y

        # 時間最近的 leader 樣本
        i = bisect.bisect_left([s[0] for s in self.leader_samples], t)
        i = min(max(i, 0), len(self.leader_samples) - 1)
        if i > 0 and abs(self.leader_samples[i - 1][0] - t) < abs(self.leader_samples[i][0] - t):
            i -= 1
        _, lx, ly, _ = self.leader_samples[i]

        gap = self.trail.path_gap(fx, fy)          # 沿軌跡間距（獨立重算）
        err = gap - self.args.gap_target
        center_dist = math.hypot(lx - fx, ly - fy)  # 直線中心距（碰撞用）
        self.rows.append((t, fx, fy, lx, ly, gap, err, center_dist))

    def elapsed(self) -> float:
        return 0.0 if self.t0 is None else self.leader_samples[-1][0] - self.t0

    def report(self) -> int:
        res = score_following(
            self.rows,
            rms_limit=self.args.rms_limit,
            collision_dist=self.args.collision_dist,
            xte_limit=self.args.xte_limit,
            settle=self.args.settle,
            leader_length=self.trail.total_length,
            min_samples=self.args.min_samples,
            min_leader_length=self.args.min_leader_length,
            leader_moving_since=self.leader_moving_since,
        )

        # 樣本不足 / leader 沒動：不予評分，絕不回 PASS（防假綠燈）
        if res['code'] == 2:
            print(f'\n❌ 不予評分：{res["reason"]}')
            print('   （防呆：資料不足時一律不判 PASS。檢查 topic/remap 有沒有接上、'
                  'duration 是否夠長、leader 是否真的有開動。）')
            return 2

        if self.args.csv:
            with open(self.args.csv, 'w', encoding='utf-8') as f:
                f.write('t,follower_x,follower_y,leader_x,leader_y,path_gap,gap_error,'
                        'center_dist,cross_track\n')
                for r, x in zip(self.rows, res['xte']):
                    f.write(','.join(f'{c:.6f}' for c in r) + f',{x:.6f}\n')

        verdict = res['code'] == 0
        print('\n' + '=' * 62)
        print('V2 驗收評分（follower 跟隨性能）')
        print('=' * 62)
        print(f'樣本數                : {len(self.rows)}（leader {len(self.leader_samples)} 筆）')
        print(f'時長                  : {self.elapsed():.1f} s')
        print(f'leader 軌跡總長       : {res["leader_length"]:.2f} m')
        print(f'目標間距              : {self.args.gap_target:.2f} m')
        print('-' * 62)
        print(f'間距誤差 RMS（全程）  : {res["rms_all"]:.3f} m   {"✅" if res["pass_rms"] else "❌"} '
              f'(判準 < {self.args.rms_limit:g})')
        print(f'間距誤差 RMS（開動後）: {res["rms_moving"]:.3f} m   （參考值，非判準）')
        print(f'間距誤差 最大絕對值   : {res["max_abs_err"]:.3f} m')
        print(f'兩車最小中心距        : {res["min_center_dist"]:.3f} m   '
              f'{"✅" if res["pass_collision"] else "❌ 碰撞"} '
              f'(判準 > {self.args.collision_dist:g} = 車長)')
        print(f'cross-track RMS       : {res["xte_rms"]:.3f} m   （settle {self.args.settle:g}s 後）')
        print(f'cross-track 最大值    : {res["xte_max"]:.3f} m   '
              f'{"✅ 未切內線" if res["pass_xte"] else "❌ 切內線"} '
              f'(判準 < {self.args.xte_limit:g})')
        if not res['xte_evaluable']:
            print('⚠ cross-track 判準在此資料上不可評：leader 軌跡總長沒有明顯大於跟車距離，'
                  'follower 全程在路徑起點後方、還沒踏上 leader 的路——此值量的是「未上路」'
                  '的幾何距離，不是切內線。判定照舊（FAIL 仍 FAIL），但勿以此診斷 follower。')
        print('-' * 62)
        print(f'結論: {"✅ PASS" if verdict else "❌ FAIL"}')
        if self.args.csv:
            print(f'CSV : {self.args.csv}')
        print('=' * 62)
        return res['code']


def main():
    parser = argparse.ArgumentParser(description='V2 follower 跟隨性能評分器')
    parser.add_argument('--leader-topic', default='/car1/odometry/filtered')
    parser.add_argument('--follower-topic', default='/car2/odometry/filtered')
    parser.add_argument('--gap0', type=float, default=1.0)
    parser.add_argument('--gap-target', type=float, default=1.2)
    parser.add_argument('--rms-limit', type=float, default=0.3)
    parser.add_argument('--collision-dist', type=float, default=0.337,
                        help='兩車中心距低於此值視為碰撞（預設＝車長 0.337 m）')
    parser.add_argument('--xte-limit', type=float, default=0.15,
                        help='cross-track error 上限，超過視為切內線（預設 0.15 m ≈ 80%% 車寬）')
    parser.add_argument('--settle', type=float, default=10.0,
                        help='前幾秒不計 cross-track（follower 尚未踏上 leader 路徑）')
    parser.add_argument('--min-samples', type=int, default=100,
                        help='配對樣本少於此值不予評分，避免幾筆資料誤判 PASS（防呆）')
    parser.add_argument('--min-leader-length', type=float, default=1.0,
                        help='leader 軌跡總長低於此值視為沒動、不予評分（防呆）')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--csv', default='')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ScoreFollow(args)
    # wall-clock 上限：若 topic/remap 沒接上、leader 一筆都沒進來，node.t0 永遠是 None，
    # 原本的 `elapsed() >= duration` 判斷永遠不成立會無限掛住。用真實時鐘兜底保證會退出。
    deadline = time.monotonic() + args.duration + 5.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.t0 is not None and node.elapsed() >= args.duration:
                break
            if time.monotonic() > deadline:
                node.get_logger().warn(
                    'wall-clock 逾時（duration+5s 內沒收滿資料）——多半是 topic/remap 沒接上，'
                    '直接進評分並會判「不予評分」。')
                break
    except KeyboardInterrupt:
        pass

    code = node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
