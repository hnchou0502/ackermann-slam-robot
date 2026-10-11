# 部署環境版本記錄

最後更新：2026-07-09（從 Pi 實機匯出）

## 平台
- Raspberry Pi 5 + Ubuntu Server 24.04 (noble)
- ROS 2 **Jazzy**
- 電源：官方 27W（非官方電源會讓 Pi5 對 USB 限流/狂報 over-current，光達不穩）

## ROS 套件版本
| 套件 | 版本 | 來源 |
|---|---|---|
| slam_toolbox | 2.8.5-1noble.20260614 | apt (ros-jazzy-slam-toolbox) |
| foxglove_bridge | 3.4.1-1noble.20260612 | apt (ros-jazzy-foxglove-bridge) |
| robot_localization (EKF) | apt (ros-jazzy-robot-localization) | — |
| teleop_twist_keyboard | apt | 開車用 |

## 光達（外部 clone，不在本 repo）
- **YDLidar-SDK 1.2.20**（Pi `~/YDLidar-SDK`，`sudo make install`）
- **ydlidar_ros2_driver** commit `6b9f249`（Pi `~/ros2_ws/src/`，外部 clone）
- 兩支必要補丁（在本 repo `ros2_ws/tools/`）：
  1. `patch_ydlidar_jazzy.py`：補 Jazzy 移除的單參數 `declare_parameter`（26 處）
  2. `patch_ydlidar_sdk_dtr.py`：修 SDK `startScan()` 順序 bug（這顆 2018 X4/FW1.5 馬達嚴格靠 DTR）
- 重抓 SDK 要重跑補丁 + `make && sudo make install`；ROS 驅動要清乾淨重編（`rm -rf build/install/ydlidar_ros2_driver` 再 colcon）。

## 硬體
- 光達：YDLIDAR X4，2018 版，FW1.5，Serial 2018102000004088，掃描率 ~7.9Hz
- 下位機：STM32F103RC（ERF01 v3.0 擴展板），韌體見 firmware/
- 序列埠：udev 固定命名（見 99-slamcar.rules）
  - STM32 (CH340 1a86:7523) → /dev/stm32
  - YDLIDAR (CP2102 10c4:ea60) → /dev/ydlidar

## 網路（憑證不在此，見機器本地 memory）
- SSH 走 Tailscale（最穩）。詳細連線/WiFi 憑證僅存操作者本機 memory，不進本 repo。
