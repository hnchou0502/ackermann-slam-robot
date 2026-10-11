# 換機／二號車重建流程（Raspberry Pi 5 + ROS 2 Jazzy）

最後更新：2026-07-09。版本號見 [`deploy/VERSIONS.md`](../deploy/VERSIONS.md)。
目標：一台全新 Pi 從零到能建圖。約 1–2 小時（多在編譯與光達補丁）。

## 0. 平台
- Raspberry Pi 5 + Ubuntu Server 24.04（noble）+ ROS 2 Jazzy
- **電源必須用官方 27W**：非官方電源會讓 Pi5 對 USB 限流/狂報 over-current，光達不穩。

## 1. 系統依賴
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-foxglove-bridge \
  python3-colcon-common-extensions cmake build-essential git
```

## 2. 取得本 repo
```bash
git clone <repo-url> ~/slam-car
mkdir -p ~/ros2_ws/src
cp -r ~/slam-car/ros2_ws/src/stm32_bridge ~/ros2_ws/src/
```

## 3. udev 固定序列埠命名（關鍵，先做）
```bash
sudo cp ~/slam-car/deploy/99-slamcar.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
# 插上 STM32 與光達，確認：
ls -l /dev/stm32 /dev/ydlidar
```
不設的話多裝置時 `ttyUSBx` 編號會亂跳（插光達會把 STM32 從 ttyUSB0 擠走）。

## 4. 光達驅動（外部 clone + 兩支必要補丁）
```bash
# 4a. YDLidar-SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK ~/YDLidar-SDK
cd ~/YDLidar-SDK
python3 ~/slam-car/ros2_ws/tools/patch_ydlidar_sdk_dtr.py    # 修 startScan() 順序 bug（2018 X4/FW1.5 馬達靠 DTR）
mkdir build && cd build && cmake .. && make && sudo make install

# 4b. ROS2 驅動
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver ~/ros2_ws/src/ydlidar_ros2_driver
python3 ~/slam-car/ros2_ws/tools/patch_ydlidar_jazzy.py       # 補 Jazzy 移除的單參數 declare_parameter（26 處）
cp ~/slam-car/deploy/ydlidar_X4.yaml ~/ros2_ws/src/ydlidar_ros2_driver/params/X4.yaml
```
> ⚠ 若日後重抓 SDK：要重跑補丁 + `make && sudo make install`，且 ROS 驅動要**清乾淨重編**
> （`rm -rf ~/ros2_ws/build/ydlidar_ros2_driver ~/ros2_ws/install/ydlidar_ros2_driver` 再 colcon），否則不會 relink 到新 `.a`。

## 5. 編譯
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stm32_bridge ydlidar_ros2_driver
source install/setup.bash
```

## 6. 韌體（下位機）
STM32F103RC 韌體在 `firmware/`（STM32CubeIDE 專案）。燒錄用 mcuisp（Windows，見機器本地 memory），不在 Pi 上做。

## 7. 啟動與驗證
```bash
# 一鍵起 bridge + EKF + 光達 + 靜態 TF
ros2 launch stm32_bridge bringup.launch.py                 # 純唯讀不開車：加 enable_cmd:=false
# 驗證
ros2 topic hz /scan            # ≈ 7.9Hz
ros2 run tf2_ros tf2_echo base_link laser_frame            # 應見 [0.123, 0, 0.185]
```

## 8. 建圖（slam_toolbox）
```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p odom_frame:=odom -p base_frame:=base_link -p scan_topic:=/scan \
  -p mode:=mapping -p resolution:=0.05 -p max_laser_range:=12.0
# ⚠ Jazzy 是 lifecycle node，必須手動轉狀態：
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
# 開車（另一終端）：ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15
# 存圖：ros2 run nav2_map_server map_saver_cli -f mymap
```
建圖務必**慢開、轉彎柔和**（光達僅 7.9Hz，快動會殘影）。

## 9. 即時可視化（Foxglove）
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```
在另一台電腦用 **Foxglove Studio 桌面版**連 `ws://<pi-ip>:8765`（網頁版會被 mixed-content 擋）。3D panel 勾 `/map`+`/scan`+`/tf`，關透視看俯視。

## 10. 錄資料（離線重建用）
```bash
ros2 bag record -s mcap -o run1 /odom /imu/data /odometry/filtered /tf /tf_static /scan /cmd_vel
# 停止用 kill -TERM（Ctrl-C 對 detached recorder 有時無效）
```

## 常見問題
- **`/scan` 不通、`Failed to start`**：多半是沒跑 DTR 補丁或 SDK 未重編。這顆 2018 X4 馬達嚴格靠 DTR。
- **slam 沒建圖、`/map` 不發**：99% 是忘了 `lifecycle configure→activate`，或 scan QoS（bridge 已處理）。
- **序列埠搶埠**：確認 udev 生效、X4.yaml 的 port 覆蓋成 `/dev/ydlidar`（bringup 已覆蓋）。
