#!/usr/bin/env python3
"""里程尺度標定：互動式量測 DISTANCE_PER_COUNT 的修正值。

原理：odom 距離 = 編碼器計數 × DISTANCE_PER_COUNT。若實測「推 D_true 公尺、odom 只記
D_odom」，則正確常數 = 目前值 × (ΣD_true / ΣD_odom)。多段量測取整體比例，並檢查各段
一致性——各段比例接近＝純尺度問題（改常數即準）；離散大＝有打滑，要先查。

前提（在 Pi 上跑）：bridge 已在發 /odom（`ros2 launch stm32_bridge bringup.launch.py`，
或只起 bridge）。**手推、慢推、走直線、不轉彎**；起終點對同一基準（後輪軸心）。

用法：
  python3 calibrate_odom_scale.py --current 0.0004618
流程：每段先把車擺到起點按 Enter，推到終點、量實際距離、輸入公尺數按 Enter；
     全部量完輸入 done，腳本印出建議的新常數與一致性報告。
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomProbe(Node):
    def __init__(self):
        super().__init__('odom_probe')
        self.last = None
        self.create_subscription(Odometry, 'odom', self._cb, 10)

    def _cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.last = (p.x, p.y)

    def wait_pose(self, timeout=5.0):
        """spin 到收到至少一筆 /odom，回傳 (x,y)。"""
        t = 0.0
        while rclpy.ok() and self.last is None and t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            t += 0.1
        return self.last

    def sample(self):
        self.last = None
        return self.wait_pose()


def main():
    ap = argparse.ArgumentParser(description='里程尺度標定（互動式）')
    ap.add_argument('--current', type=float, required=True,
                    help='目前固件的 DISTANCE_PER_COUNT（main.c，如 0.0004618）')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = OdomProbe()
    print('等待 /odom…（確認 bridge 在跑）')
    if node.sample() is None:
        print('❌ 收不到 /odom，先起 bringup/bridge 再來。'); sys.exit(1)
    print('✓ 收到 /odom。手推、慢推、走直線、不轉彎；起終點對後輪軸心。\n')

    segs = []   # (d_true, d_odom)
    while True:
        cmd = input(f'[第 {len(segs)+1} 段] 車擺到起點後按 Enter（或輸入 done 結束）：').strip()
        if cmd.lower() == 'done':
            break
        x0, y0 = node.sample()
        raw = input('  推到終點、對齊後，輸入實際距離(公尺)按 Enter：').strip()
        try:
            d_true = float(raw)
            if d_true <= 0:
                raise ValueError
        except ValueError:
            print('  ⚠ 距離要是正數，這段跳過。'); continue
        x1, y1 = node.sample()
        d_odom = math.hypot(x1 - x0, y1 - y0)
        if d_odom < 1e-4:
            print('  ⚠ odom 幾乎沒動，這段跳過（車真的推了嗎？）。'); continue
        ratio = d_true / d_odom
        segs.append((d_true, d_odom))
        print(f'  記錄：實際 {d_true:.3f} m / odom {d_odom:.3f} m → 本段比例 {ratio:.4f}\n')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if len(segs) < 2:
        print('\n量測段數 < 2，資料不足以標定（至少 2 段、建議不同距離各 2~3 趟）。')
        sys.exit(1)

    sum_true = sum(s[0] for s in segs)
    sum_odom = sum(s[1] for s in segs)
    overall = sum_true / sum_odom
    new_const = args.current * overall

    ratios = [s[0] / s[1] for s in segs]
    mean_r = sum(ratios) / len(ratios)
    spread = max(ratios) - min(ratios)

    print('\n' + '=' * 56)
    print('里程尺度標定結果')
    print('=' * 56)
    print(f'段數                : {len(segs)}')
    print(f'各段比例            : ' + ', '.join(f'{r:.4f}' for r in ratios))
    print(f'整體比例(Σ真/Σodom) : {overall:.4f}')
    print(f'目前 DISTANCE_PER_COUNT : {args.current:.7f}')
    print(f'建議 DISTANCE_PER_COUNT : {new_const:.7f}  （×{overall:.4f}）')
    print('-' * 56)
    if spread > 0.03:
        print(f'⚠ 各段比例離散 {spread:.4f} > 0.03：可能有打滑/量測不一致，')
        print('   先別烧！挑離群的那段重量，或改更慢/更直的推法再測。')
    else:
        print(f'✓ 各段比例一致（離散 {spread:.4f} ≤ 0.03）＝純尺度問題，')
        print(f'   把 main.c:62 的 DISTANCE_PER_COUNT 改成 {new_const:.7f} 重烧，')
        print('   再推一趟驗證 odom ≈ 實際即完成。')
    print('=' * 56)


if __name__ == '__main__':
    main()
