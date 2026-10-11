#!/usr/bin/env python3
"""把 ydlidar_ros2_driver 修成能在 ROS2 Jazzy 編譯（2026-06-28）。

問題：驅動原始碼用了 Jazzy 已移除的「單參數」`declare_parameter("X")`（無預設值版）。
修法：每個 `declare_parameter("X")` 的下一行就是 `get_parameter("X", 變數)`，而那個變數
      前面已設好預設值。所以把預設值補進 declare → `declare_parameter("X", 變數)`。

用法（在 Pi 上，驅動 clone 完、build 之前）：
    cd ~/ros2_ws/src/ydlidar_ros2_driver
    python3 <此檔>
    cd ~/ros2_ws && colcon build --packages-select ydlidar_ros2_driver

⚠ 若重新 clone 驅動，要重跑此 patch。實測改 26 處後 build 通過。
（注意：驅動自帶的 launch 檔也用了舊 API node_executable，不能用；要直接 ros2 run。）
"""
import re

f = "src/ydlidar_ros2_driver_node.cpp"
lines = open(f).read().split("\n")
dre = re.compile(r'declare_parameter\("([^"]+)"\)')          # 只配單參數版
gre = re.compile(r'get_parameter\("([^"]+)",\s*([^)]+)\)')
patched = 0
for i in range(len(lines)):
    m = dre.search(lines[i])
    if not m:
        continue
    name = m.group(1)
    var = None
    for j in range(i + 1, min(i + 4, len(lines))):
        g = gre.search(lines[j])
        if g and g.group(1) == name:
            var = g.group(2).strip()
            break
    if var:
        lines[i] = dre.sub('declare_parameter("%s", %s)' % (name, var), lines[i])
        patched += 1
open(f, "w").write("\n".join(lines))
print("patched", patched)
