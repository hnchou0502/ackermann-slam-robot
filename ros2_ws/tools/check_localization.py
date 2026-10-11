"""量定位品質：把當前光達點雲繞車體旋轉不同角度，找最貼合地圖的角度。

建立 2026-08-11（EX1 實跑時，用戶目視發現「光達點跟牆沒對齊、整體逆時針偏」而寫）。
用來分辨兩件長得很像、成因完全不同的事：

  最佳角 ≈ 0 且吻合率高   → 定位良好（實測車剛走完一段時：0.0 度 / 98.3%）
  最佳角明顯偏離 0        → 地圖沒歪，是「當前這幀掃描」的朝向被漂移帶偏；
                            車一動 slam 就會修回來（實測靜置 4.4 分鐘後：-11 度 / 47.6%，
                            且 11 度 ÷ 4.4 分鐘 ≈ 陀螺零漂 0.042 度/秒，三方吻合）
  各角度吻合率都低        → 才是地圖本身有問題

⚠ 不要改用「地圖牆格的方向直方圖」判斷地圖有無畸變：占據網格上的斜牆是階梯狀，
  會製造 45 度假峰，牆格少時完全不可信（2026-08-11 首版就是這樣誤判成「地圖畸變」）。

用法（Pi 上，需 bringup + slam 正在跑）：python3 check_localization.py
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

MAP_QOS = QoSProfile(depth=1)
MAP_QOS.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
MAP_QOS.reliability = QoSReliabilityPolicy.RELIABLE


class C(Node):
    def __init__(self):
        super().__init__('angle_sweep')
        self.grid = None
        self.scan = None
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'grid', m), MAP_QOS)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 5)
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)


def main():
    rclpy.init()
    c = C()
    for _ in range(60):
        rclpy.spin_once(c, timeout_sec=0.2)
    for _ in range(80):
        rclpy.spin_once(c, timeout_sec=0.2)
        if c.grid is not None and c.scan is not None:
            break
    g, sc = c.grid, c.scan
    if g is None or sc is None:
        print('資料不齊')
        return

    res, ox, oy = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y
    W, H = g.info.width, g.info.height
    # TF buffer 暖機＋重試：剛建立的 listener 查不到，或 latest common time 落在
    # cache 之外時會拋 Extrapolation；等到真的查得到再往下（2026-08-11 踩三次）。
    tf = None
    for _ in range(60):
        try:
            tf = c.buf.lookup_transform('map', 'laser_frame', Time())
            break
        except Exception:
            rclpy.spin_once(c, timeout_sec=0.2)
    if tf is None:
        print('等不到 map→laser_frame 變換')
        rclpy.shutdown()
        return
    lx, ly = tf.transform.translation.x, tf.transform.translation.y
    q = tf.transform.rotation
    lyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    pts = []
    for k, r in enumerate(sc.ranges):
        if not (sc.range_min < r < sc.range_max) or math.isinf(r) or math.isnan(r):
            continue
        pts.append((sc.angle_min + k * sc.angle_increment, r))

    def score(dth):
        hit = 0
        for a0, r in pts:
            a = a0 + lyaw + dth
            mx, my = lx + r * math.cos(a), ly + r * math.sin(a)
            i0, j0 = int((mx - ox) / res), int((my - oy) / res)
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    i, j = i0 + di, j0 + dj
                    if 0 <= i < W and 0 <= j < H and g.data[j * W + i] > 50:
                        hit += 1
                        break
                else:
                    continue
                break
        return 100.0 * hit / max(len(pts), 1)

    print('光達點數 =', len(pts), ' 地圖牆格數 =', sum(1 for v in g.data if v > 50))
    print()
    print('旋轉角度   吻合率')
    best = (None, -1)
    results = []
    d = -25.0
    while d <= 25.001:
        s = score(math.radians(d))
        results.append((d, s))
        if s > best[1]:
            best = (d, s)
        d += 1.0
    # 只印每 2 度一行，避免洗版
    for d, s in results:
        if abs(d % 2) < 0.01:
            bar = '#' * int(s / 2)
            mark = '  <=== 最佳' if abs(d - best[0]) < 0.01 else ''
            print('%+6.1f 度   %5.1f%%  %s%s' % (d, s, bar, mark))

    print()
    print('最佳旋轉角 = %+.1f 度（吻合率 %.1f%%），未旋轉時 = %.1f%%'
          % (best[0], best[1], score(0.0)))
    print()
    print('判讀：')
    print('  最佳角明顯不是 0 且旋轉後吻合率大幅提升 → 地圖沒歪，是當前掃描的朝向被漂移帶偏')
    print('  各角度吻合率都低 → 地圖本身有問題')
    rclpy.shutdown()


main()
