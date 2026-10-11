#!/usr/bin/env python3
"""從 bag 算車子實際走過的路徑長度與航向變化量，判斷是不是真的繞了一圈。
用法：python3 bag_path.py <bag 目錄>
"""
import sys
import math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
    rosbag2_py.ConverterOptions("", ""),
)
types = {t.name: t.type for t in reader.get_all_topics_and_types()}

odom_msgs = []
cmd_count = 0
scan_count = 0
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == "/odometry/filtered":
        m = deserialize_message(data, get_message(types[topic]))
        odom_msgs.append((t, m))
    elif topic == "/cmd_vel":
        cmd_count += 1
    elif topic == "/scan":
        scan_count += 1

if not odom_msgs:
    print("bag 裡沒有 /odometry/filtered")
    sys.exit(1)

path = 0.0
xs, ys = [], []
prev = None
yaw_total = 0.0
prev_yaw = None
max_speed = 0.0
for t, m in odom_msgs:
    p = m.pose.pose.position
    xs.append(p.x)
    ys.append(p.y)
    if prev is not None:
        path += math.hypot(p.x - prev[0], p.y - prev[1])
    prev = (p.x, p.y)

    q = m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    if prev_yaw is not None:
        d = yaw - prev_yaw
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        yaw_total += abs(d)
    prev_yaw = yaw
    max_speed = max(max_speed, abs(m.twist.twist.linear.x))

t0 = odom_msgs[0][0] / 1e9
t1 = odom_msgs[-1][0] / 1e9
span = t1 - t0

start = (xs[0], ys[0])
end = (xs[-1], ys[-1])
disp = math.hypot(end[0] - start[0], end[1] - start[1])

print(f"錄製時長      {span:.1f} s")
print(f"/scan 筆數    {scan_count}（約 {scan_count/span:.1f} Hz）")
print(f"/cmd_vel 筆數 {cmd_count}")
print()
print(f"起點          ({start[0]:+.3f}, {start[1]:+.3f})")
print(f"終點          ({end[0]:+.3f}, {end[1]:+.3f})")
print(f"直線位移      {disp:.3f} m")
print(f"**累計路徑長  {path:.3f} m**")
print(f"累計轉向量    {math.degrees(yaw_total):.0f}°")
print(f"最高速度      {max_speed:.3f} m/s")
print()
print(f"走訪範圍      x {min(xs):+.2f} ~ {max(xs):+.2f} m（跨度 {max(xs)-min(xs):.2f}）")
print(f"              y {min(ys):+.2f} ~ {max(ys):+.2f} m（跨度 {max(ys)-min(ys):.2f}）")
print()
if path < 0.5:
    print("→ 車子幾乎沒動。這張圖等於單點掃描，**不能用來證明疊圖已解決**。")
elif path < 3.0:
    print("→ 移動距離偏短，只掃到局部；疊圖結論證據力薄弱。")
else:
    print("→ 有實際繞行，地圖具備多視角，疊圖結論成立。")
