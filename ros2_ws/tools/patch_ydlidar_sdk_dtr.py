#!/usr/bin/env python3
"""修 YDLidar-SDK 的「啟動掃描」順序 bug，讓「靠 DTR 開關馬達」的舊款 X4 能啟動（2026-06-29）。

== 症狀 ==
ydlidar_ros2_driver（與原廠 tri_test 範例）連得上、讀得到型號（Model X4, FW 1.5），
但一律卡在 `Failed to start the lidar`，/scan 完全發不出來。馬達只「轉一下就停」。

== 根因（這顆 2018 年 X4、韌體 1.5 特有）==
這顆 X4 的馬達「嚴格」靠序列埠 DTR 線開關：DTR=True → 馬達轉；DTR=False → 馬達停。
（已用底層 pyserial 驗證：dtr=True 時送 0xA5 0x60 可讀到 42921 bytes 正確掃描封包 a55a；
  dtr=False 時只有 7 bytes、馬達不轉。）

但 SDK `src/YDlidarDriver.cpp` 的 `startScan()` 順序是：
    stop();                       // → stopMotor() → clearDTR() → DTR=False → 馬達關掉
    sendCmd(LIDAR_CMD_SCAN);
    waitResp(MEASUREMENT, ...);   // 等掃描資料……但馬達是關的，永遠等不到 → 逾時 fail
    ...
    startMotor();                 // 開馬達(DTR=True) 卻排在最後，根本到不了
對「開機就一直轉、不甩 DTR」的新款 X4 沒事（waitResp 拿得到資料）；
對這顆「DTR 才轉」的舊單位就變成雞生蛋死結 → Failed to start。
（用戶證言佐證：同一顆 X4 在 ROS1 + 樹莓派 4 的舊驅動上「很快就會轉」。）

== 修法 ==
把 `startScan()` 裡的 `if (isSupportMotorCtrl(model)) startMotor();`
從 waitResp「之後」移到「之前」（緊接 delay(30) 後），讓馬達先轉起來
（startMotor 內含 delay(500) 等它到速）再送掃描指令、等資料。

== 用法（在 Pi 上，YDLidar-SDK 已 clone+build 過）==
    python3 patch_ydlidar_sdk_dtr.py /home/<user>/YDLidar-SDK/src/YDlidarDriver.cpp
    cd /home/<user>/YDLidar-SDK/build && make -j4 && sudo make install
    # 關鍵：ROS 驅動要「清乾淨」重編才會連結到新 SDK（colcon 偵測不到 .a 變動）：
    cd ~/ros2_ws && rm -rf build/ydlidar_ros2_driver install/ydlidar_ros2_driver
    colcon build --packages-select ydlidar_ros2_driver

驗證：`ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
        --params-file <X4.yaml port=/dev/ydlidar>` → `ros2 topic hz /scan` 應約 7.9Hz。

⚠ 若重新 clone SDK，要重跑此 patch。另把 X4.yaml 的 fixed_resolution 設 false
  （否則 "Real points 631 > fixed points 530" 警告會丟點）。
"""
import sys

ANCHOR_INSERT = "  checkTransDelay();\n  delay(30);\n  {\n"
REPLACE_INSERT = (
    "  checkTransDelay();\n  delay(30);\n"
    "  if (isSupportMotorCtrl(model))\n    startMotor();\n  {\n"
)
ANCHOR_REMOVE = (
    "  if (isSupportMotorCtrl(model))\n    startMotor();\n\n  m_isScanning = true;"
)
REPLACE_REMOVE = "  m_isScanning = true;"


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "src/YDlidarDriver.cpp"
    s = open(path).read()
    n1, n2 = s.count(ANCHOR_INSERT), s.count(ANCHOR_REMOVE)
    print("anchor_insert=%d anchor_remove=%d" % (n1, n2))
    if n1 != 1 or n2 != 1:
        print("錨點不唯一，SDK 版本可能不同，請手動確認 startScan()。中止。")
        sys.exit(1)
    s = s.replace(ANCHOR_INSERT, REPLACE_INSERT).replace(ANCHOR_REMOVE, REPLACE_REMOVE)
    open(path, "w").write(s)
    print("PATCH_OK ->", path)


if __name__ == "__main__":
    main()
