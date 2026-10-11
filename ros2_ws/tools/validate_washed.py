#!/usr/bin/env python3
"""驗證洗過的 bag：每個輸出一行機器可查的結論。

檢查項：
  1. 第一筆 /odometry/filtered 位置 ≈ (0,0)、航向 ≈ 0（起點歸零成立）
  2. 有效輸出頻率（以 header.stamp 的 sim-time 跨度算）≥ 15Hz
     （EKF 名目 30Hz；計時器跟 /clock tick 走，回放參數會壓低它。
      V3 消費端 v2v_relay 只要 10Hz，15Hz 留了安全餘量。
      ⚠ 不可用「錄檔收到時刻」算跨度——3 倍速回放下那是 sim 跨度的 1/3，會誤判。）
  3. 全程無 NaN
  4. 路徑總長 vs 原 bag 的 /odometry/filtered 路徑總長（同一 vx 積分，量級應相近）
用法：python3 validate_washed.py <原bag> <洗後bag> [<原bag> <洗後bag> ...]
"""
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

Odom = get_message('nav_msgs/msg/Odometry')


def read_odom(bag_dir):
    """回傳 (first_pose, n, path_len, has_nan, t_span)。

    t_span 用 **header.stamp**（訊息自己的 sim 時間），不是錄檔收到時刻——
    3 倍速回放下收到時刻的跨度只有 sim 跨度的 1/3，拿它算頻率會高估 3 倍。
    """
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    first = None
    n = 0
    path = 0.0
    has_nan = False
    px = py = None
    t_first = t_last = None
    while r.has_next():
        topic, data, t = r.read_next()
        if topic != '/odometry/filtered':
            continue
        m = deserialize_message(data, Odom)
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        if math.isnan(p.x) or math.isnan(p.y):
            has_nan = True
            continue
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if first is None:
            first = (p.x, p.y, yaw)
            t_first = stamp
        else:
            path += math.hypot(p.x - px, p.y - py)
        px, py = p.x, p.y
        t_last = stamp
        n += 1
    span = (t_last - t_first) if (t_first is not None and n > 1) else 0.0
    return first, n, path, has_nan, span


def main():
    pairs = list(zip(sys.argv[1::2], sys.argv[2::2]))
    all_ok = True
    for orig_dir, washed_dir in pairs:
        name = washed_dir.rstrip('/').split('/')[-1]
        try:
            w_first, w_n, w_path, w_nan, w_span = read_odom(washed_dir)
            _, o_n, o_path, _, o_span = read_odom(orig_dir)
        except Exception as e:
            print(f'{name}: ERR {e}')
            all_ok = False
            continue
        if w_first is None:
            print(f'{name}: FAIL 洗後 bag 沒有 /odometry/filtered')
            all_ok = False
            continue
        x0, y0, yaw0 = w_first
        origin_ok = math.hypot(x0, y0) < 0.05 and abs(math.degrees(yaw0)) < 2.0
        # 有效輸出頻率（sim time）：V3 消費端要 10Hz，判準 ≥15Hz 留餘量
        rate = (w_n - 1) / w_span if w_span > 0 else 0.0
        rate_ok = rate >= 15.0
        span_ok = o_span > 0 and abs(w_span - o_span) / o_span < 0.10
        path_ok = o_path > 0 and abs(w_path - o_path) / o_path < 0.25
        ok = origin_ok and rate_ok and span_ok and (not w_nan) and path_ok
        all_ok = all_ok and ok
        print(f'{name}: {"OK" if ok else "FAIL"}  '
              f'first=({x0:.3f},{y0:.3f},{math.degrees(yaw0):.1f}deg){"✓" if origin_ok else "✗"}  '
              f'rate={rate:.1f}Hz{"✓" if rate_ok else "✗"}  '
              f'nan={"有✗" if w_nan else "無✓"}  '
              f'path={w_path:.1f}m(原{o_path:.1f}m){"✓" if path_ok else "✗"}  '
              f'span={w_span:.0f}s(原{o_span:.0f}s){"✓" if span_ok else "✗"}')
    print('ALL_OK' if all_ok else 'SOME_FAIL')


if __name__ == '__main__':
    main()
