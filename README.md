# Ackermann SLAM Robot

A small Ackermann-steering robot built from the motor driver up, first as my undergraduate
capstone (single robot, ROS 1) and then extended on my own into a **two-robot testbed for
collaborative mapping** (ROS 2).

**Author:** Hao Ning Chou (鄒昊寧), B.S. in Electronic Engineering, Chang Gung University, Taiwan (2026)

> Most code comments and the capstone report are written in Traditional Chinese.

---

## 1. Capstone project (2025, ROS 1)

*Layered-Control Ackermann SLAM Robot*: a two-person team project; I carried out the design,
implementation, and experiments. Report: [`docs/capstone_report_zh.pdf`](docs/capstone_report_zh.pdf) (Chinese).

The robot separates real-time control from perception and planning:

```
[ROS 1 Noetic on Raspberry Pi 4]  gmapping · AMCL · move_base + TEB local planner
        │  /cmd_vel → custom integer UART protocol (115200 baud, values ×1000)
[STM32F103 @ 50 Hz]  incremental PID (rear motors) · bicycle-model steering angle → servo
        │  wheel encoders + IMU → feedback frames
[YDLIDAR X4 2D LiDAR]  → mapping and localization
```

- **Firmware (my main part):** incremental PID, Ackermann bicycle-model steering, 50 Hz control
  loop, 500 ms communication watchdog.
- **Custom UART protocol instead of `rosserial`:** the F103 did not have the resources for it, so
  all quantities are sent as integers ×1000 and parsed by hand on the MCU
  ([`docs/protocol.md`](docs/protocol.md)).
- **Result on the real robot:** straight-line speed error below 2.5% at 0.1, 0.3, and 0.5 m/s;
  indoor mapping, AMCL localization, and goal navigation with obstacle avoidance.

Capstone ROS 1 package: [`ros/stm32_bridge/`](ros/stm32_bridge/) (archived).

## 2. Extension after graduation (2026, ROS 2)

### Single robot
- Ported to **Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy**.
- `robot_localization` EKF (linear velocity from the wheels, yaw rate from the IMU),
  `slam_toolbox` mapping, Nav2 navigation (13 of 13 goal trials succeeded across the two robots).
- Found and patched a start-up deadlock in the YDLIDAR X4 vendor SDK
  ([`ros2_ws/tools/patch_ydlidar_sdk_dtr.py`](ros2_ws/tools/patch_ydlidar_sdk_dtr.py)).
- Odometry scale calibrated from 11.8% short to 0.29% error.
- Main LiDAR upgraded to an RPLIDAR C1 (10 Hz, 720 points per scan).
- Multi-session mapping: a second session continued the first pose graph with 11 cross-session
  loop closures and kept 98.1% of the first session's walls.

### Two-robot platform
- **Heterogeneous sensing:** robot 1 uses an RPLIDAR C1, robot 2 an RPLIDAR A1M8 (different scan
  rate, point count, and mounting height). Robot 2 also carries a depth camera, which is **not yet
  integrated**.
- **V2V link:** namespaced ROS 2 setup; `v2v_relay` exchanges state at 10 Hz with send-time stamps.
  With chrony time sync (offset about 0.5 ms), the measured one-way latency over a single indoor
  Wi-Fi router was about 13 ms on average and 52 ms at most, with no losses in 150 samples.
- **Leader–follower control (pure pursuit):** validated in simulation running on the two robots'
  onboard computers over real Wi-Fi (spacing RMS 0.018 m) and against a recorded real-vehicle
  trajectory (spacing RMS 0.022 m). Two physical robots following each other has not been tested yet.
- **Offline map merging with unknown initial pose:** each robot builds its own map; the relative
  transform is found by a brute-force FFT cross-correlation search over rotation and translation
  ([`ros2_ws/tools/auto_align_maps.py`](ros2_ws/tools/auto_align_maps.py),
  [`merge_maps_offline.py`](ros2_ws/tools/merge_maps_offline.py)).

  | Scene | Walls within 5 cm after alignment | Unaligned baseline |
  |---|---|---|
  | Living room (complementary coverage) | **80.3%** | 24.0% |
  | Room (full overlap) | **93.4%** | 42.2% |

- **Negative result:** image-feature alignment (SIFT/ORB + RANSAC,
  [`opencv_align_maps.py`](ros2_ws/tools/opencv_align_maps.py)) scored 43.0% / 47.0% in the room
  (about the same as no alignment) and degenerated in the living room. Occupancy grids are sparse
  line drawings, so local features are not distinctive.
- **Live merging, bench test:** `map_merge_live` aligned a map sent over Wi-Fi in 5.9 s
  (yaw +33.0°, matching the offline result of +32.9°). It refuses to publish when the score is low,
  the alignment is ambiguous (for example a long straight wall), or too many free cells conflict;
  the thresholds were calibrated on recorded data.

### In progress / next
- Autonomous frontier exploration (in progress).
- Merging maps **while both robots drive**, under realistic V2V delay, packet loss, and bandwidth
  limits. This is the research I plan to do next; it has not been done yet.

## Repository layout

| Path | Contents |
|---|---|
| `firmware/` | STM32CubeIDE project for the STM32F103 (`Core/Src/main.c`) |
| `ros/stm32_bridge/` | Capstone ROS 1 package (gmapping / navigation launch files, archived) |
| `ros2_ws/src/stm32_bridge/` | Current ROS 2 package: serial bridge, EKF config, V2V relay, follower, live map merging, frontier explorer, unit tests |
| `ros2_ws/tools/` | Calibration, map comparison and merging, analysis scripts |
| `deploy/` | udev rules, LiDAR parameters, version notes |
| `pi-tools/` | Small hardware test scripts |
| `docs/` | Serial protocol, setup guide, capstone report |

Recorded datasets (rosbags, maps) are not included.

## Hardware

| Part | Component |
|---|---|
| Upper computer | Raspberry Pi 5 (Raspberry Pi 4 in the capstone) |
| MCU | STM32F103RC, 72 MHz |
| Drive | 2 × DC gear motors with Hall encoders (rear differential) |
| Steering | Servo with front Ackermann linkage (wheelbase about 0.235 m) |
| Sensing | 2D LiDAR (YDLIDAR X4 → RPLIDAR C1; RPLIDAR A1M8 on robot 2), ICM-20948 IMU |

## Setup

See [`docs/SETUP.md`](docs/SETUP.md). The unit tests run without ROS (needs `numpy`, `pyyaml`, `pytest`;
260 passed, 2 skipped on 2026-10-11):
`PYTHONPATH=ros2_ws/src/stm32_bridge python -m pytest ros2_ws/src/stm32_bridge/test/`.

## License

MIT, see [LICENSE](LICENSE). Files under `firmware/Drivers` and `firmware/Middlewares` are
STMicroelectronics code and keep their original license headers.
