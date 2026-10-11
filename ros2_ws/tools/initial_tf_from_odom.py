#!/usr/bin/env python3
"""從兩段建圖時錄的新 EKF 里程，算 B 圖座標系在 A 圖座標系中的初始位姿。

原理：B 段的 EKF 從零起算（原點＝bag 時刻 t_ref 的真實車位、x 軸＝當時車頭方向），
A 段的 EKF 涵蓋 t_ref（A 段播到 225s > B 段起點 195s），
所以「A 段里程在 t_ref 的位姿」＝ B 系 → A 系的變換。
（這就是雙車劇本裡「已知初始相對位姿」的角色，真雙車時改用捲尺與對齊治具取得。）

用法: cmpre_tf.py <A的odom bag目錄> <B的odom bag目錄>
輸出: 最後一行 "TF tx ty yaw_rad"
"""
import sys, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def read_odom(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    out = []
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic != '/odometry/filtered':
            continue
        m = deserialize_message(data, Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        out.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    return out

A = read_odom(sys.argv[1])
B = read_odom(sys.argv[2])
assert A and B, f'empty odom: A={len(A)} B={len(B)}'
t_ref = B[0][0]
print(f'B 首筆: t={t_ref:.3f} pose=({B[0][1]:.4f},{B[0][2]:.4f},{math.degrees(B[0][3]):.2f}deg)  ← 應接近 0')
print(f'A 範圍: t=[{A[0][0]:.3f}, {A[-1][0]:.3f}] {len(A)} 筆')
assert A[0][0] <= t_ref <= A[-1][0], 'B 起點不在 A 的時間範圍內'

# 線性內插 A 在 t_ref 的位姿
import bisect
ts = [p[0] for p in A]
i = bisect.bisect_left(ts, t_ref)
if i == 0:
    tx, ty, yaw = A[0][1:]
else:
    (t0, x0, y0, w0), (t1, x1, y1, w1) = A[i-1], A[min(i, len(A)-1)]
    f = 0.0 if t1 == t0 else (t_ref - t0) / (t1 - t0)
    tx = x0 + f*(x1-x0); ty = y0 + f*(y1-y0)
    dw = math.atan2(math.sin(w1-w0), math.cos(w1-w0))
    yaw = w0 + f*dw
print(f'A 在 t_ref 的位姿: ({tx:.4f}, {ty:.4f}, {math.degrees(yaw):.2f}deg)')
print(f'TF {tx:.6f} {ty:.6f} {yaw:.6f}')
