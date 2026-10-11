#!/usr/bin/env python3
"""從洗過的 bag 裁出「純前進窗口」當 V3 的 leader bag（離線，秒級完成）。

一次做三件事（Pi 上跑，需要 rosbag2_py）：
  1. 裁切：只保留 [start, start+duration] 秒（以第一筆 header.stamp 起算）的
     /odometry/filtered——窗口用 wash 後的掃描選「無倒車、距離最長」段
     （leader 禁倒車是跟隨設計的硬前提，docs/design/v2v_architecture.md §4 D6）。
  2. 歸零：窗口第一筆位姿經剛體變換映到 (0,0,0°)，之後所有位姿隨之旋轉平移
     ——V3 消費端（follower/score_follow）用固定 origin=(0,0,0)，起點必須真的在原點。
     twist 是車體座標，不受世界系旋轉影響，原樣保留。
  3. 重排時間軸：新 bag 的「播放時刻」改用 header.stamp——
     ⚠ wash_bag.sh 用 3 倍速回放錄的 bag，播放節奏被壓成 3 倍速（時長 1/3、
     位置變 3 倍速前進，但 twist 速度值不變 → follower 速度前饋脫鉤，
     2026-07-11 V3 首跑 FAIL 的主因之一）。重排後回到真實 1 倍速。

用法（在 Pi）：
  python3 trim_bag_forward.py <washed_bag> <out_bag> <start秒> <duration秒>
輸出末尾附驗證摘要（起點/筆數/時長/路徑長/頻率），肉眼核對。
"""
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

Odom = get_message('nav_msgs/msg/Odometry')
TOPIC = '/odometry/filtered'


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


def main():
    src, dst, start, duration = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name=TOPIC, type='nav_msgs/msg/Odometry', serialization_format='cdr'))

    t_base = None            # 原 bag 第一筆 header.stamp
    origin = None            # 窗口第一筆位姿 (x0, y0, yaw0)
    n = 0
    path = 0.0
    px = py = None
    first_pose = None
    s_first = s_last = None

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != TOPIC:
            continue
        m = deserialize_message(data, Odom)
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t_base is None:
            t_base = s
        rel = s - t_base
        if rel < start:
            continue
        if rel > start + duration:
            break

        p = m.pose.pose.position
        if origin is None:
            origin = (p.x, p.y, yaw_of(m.pose.pose.orientation))
        x0, y0, yaw0 = origin
        # 剛體變換：窗口起點 -> (0,0,0°)
        dx, dy = p.x - x0, p.y - y0
        c, si = math.cos(-yaw0), math.sin(-yaw0)
        nx, ny = dx * c - dy * si, dx * si + dy * c
        nyaw = yaw_of(m.pose.pose.orientation) - yaw0
        m.pose.pose.position.x = nx
        m.pose.pose.position.y = ny
        m.pose.pose.orientation.x = 0.0
        m.pose.pose.orientation.y = 0.0
        m.pose.pose.orientation.z = math.sin(nyaw / 2.0)
        m.pose.pose.orientation.w = math.cos(nyaw / 2.0)

        # 播放時刻 = header.stamp（奈秒）——恢復真實 1 倍速節奏
        t_play = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        writer.write(TOPIC, serialize_message(m), t_play)

        if first_pose is None:
            first_pose = (nx, ny, math.degrees(nyaw))
            s_first = s
        else:
            path += math.hypot(nx - px, ny - py)
        px, py = nx, ny
        s_last = s
        n += 1

    del writer   # flush + finalize metadata
    span = (s_last - s_first) if n > 1 else 0.0
    rate = (n - 1) / span if span > 0 else 0.0
    print(f'TRIM_OK {dst}')
    print(f'  first=({first_pose[0]:.3f},{first_pose[1]:.3f},{first_pose[2]:.1f}deg)  '
          f'n={n}  span={span:.1f}s  path={path:.2f}m  rate={rate:.1f}Hz')


if __name__ == '__main__':
    main()
