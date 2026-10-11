# UART 通訊協定規格（STM32 ↔ Raspberry Pi）

最後更新：2026-07-09
本檔由 `firmware/Core/Src/main.c` 與 `ros2_ws/src/stm32_bridge/stm32_bridge/protocol.py` 逐欄萃取，**以程式碼為準**。

## 實體層
- 介面：USART1，**115200 baud**、8N1
- 線路：Pi `/dev/stm32`（CH340，udev 固定命名）↔ STM32 USART1
- 編碼：**純 ASCII 整數**（放棄 rosserial／浮點；物理量一律 ×1000 以整數傳，收端 ÷1000 還原）
- 這是自訂協定，非 rosmaster/rosserial。

---

## 上行 `$FBK`（STM32 → Pi）

STM32 回報輪速、PWM、IMU。50Hz 控制迴路中每 2 次送一次 → **實際約 25Hz**。

**格式**（`main.c:364`）：
```
$FBK,vL,vR,pwmL,pwmR,gyroZ,yaw,#\r\n
```

| 欄位 | 內容 | 韌體來源 (main.c:365-370) | 單位（傳輸） | 還原 (protocol.py) |
|---|---|---|---|---|
| 1 vL | 左輪速 | `g_speed_m2_filtered_mps × 1000` | mm/s 整數 | ÷1000 → m/s |
| 2 vR | 右輪速 | `g_speed_m1_filtered_mps × 1000` | mm/s 整數 | ÷1000 → m/s |
| 3 pwmL | 左輪 PWM | `g_motor2_pwm_out` | duty count 0–19999 | 原值 |
| 4 pwmR | 右輪 PWM | `g_motor1_pwm_out` | duty count 0–19999 | 原值 |
| 5 gyroZ | Z 軸角速度 | `g_gyro_z_dps × 1000` | (deg/s)×1000 整數 | ÷1000×π/180 → rad/s |
| 6 yaw | 積分航向 | `g_yaw_angle × 1000` | deg×1000 整數 | ÷1000×π/180 → rad |

> ⚠ **馬達編號對映**：M2＝左輪、M1＝右輪（見上表韌體來源）。
> **相容性**：舊 5 欄格式 `$FBK,vL,vR,pwmL,pwmR,#` 仍可解析（`protocol.py:57` 判 `len(parts)>=7` 才取 IMU 兩欄）。既有只讀 parts[1..4] 的工具向後相容。
> gyroZ/yaw 由 ICM-20948 在 STM32 內積分後上傳；2026-06-28 起韌體才有這兩欄。

Pi 端 bridge 收到後：vL/vR → `/odom`（純輪速推算）；gyroZ/yaw → `/imu/data`（同時戳）。EKF 融合後出 `/odometry/filtered`。

---

## 下行 `$CMD`（Pi → STM32）

Pi 下達速度指令。bridge 以 20Hz 重送（餵韌體 500ms 看門狗）。

**格式**（`protocol.py:73` / 解析 `main.c:383`）：
```
$CMD,vx,wz,#\n
```

| 欄位 | 內容 | 單位（傳輸） | 韌體還原 (main.c:397-398) |
|---|---|---|---|
| 1 vx | 線速度 | (m/s)×1000 整數 | `g_target_speed = vx/1000` (m/s) |
| 2 wz | 角速度 | (rad/s)×1000 整數 | `g_target_angular = wz/1000` (rad/s) |

範例：`$CMD,500,-200,#\n` = 前進 0.5 m/s、順時針 0.2 rad/s。

**韌體解析**（`main.c:383-400`）：`strncmp("$CMD",4)` 起頭 → `strchr` 找逗號 → `atoi`（非 sscanf）。容忍尾綴 `,#`。

**方向慣例**（2026-07-08 實機驗證）：`wz>0`＝左轉（前輪左偏），符合 ROS REP-103 右手定則。指令鏈與 IMU 回授鏈符號皆已驗證正確。

---

## 安全機制
| 機制 | 位置 | 行為 |
|---|---|---|
| 速度夾限 | bridge | vx ±0.5 m/s、wz ±2.0 rad/s |
| 阿克曼保護 | bridge / 韌體 | 低速時 wz 歸零（車不能原地旋轉） |
| dead-man | bridge (0.5s) | 太久沒收到 `/cmd_vel` → 送停車 |
| 通訊看門狗 | 韌體 (`main.c:287`, 500ms) | 太久沒收到 `$CMD` → 強制停車 |

> ⚠ 目前**無 checksum**（室內短線 UART 可接受，列為觀察項）。
