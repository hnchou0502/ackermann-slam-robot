#!/usr/bin/env python3
"""Slamtec 光達裸協定驗貨：GET_INFO / GET_HEALTH，不經 ROS。

先確認「硬體本身活著」再去查驅動——驅動報錯時分不出是硬體壞、線鬆、還是波特率給錯。

用法：python3 lidar_probe.py [port] [baud]
  C1    ：python3 lidar_probe.py /dev/rplidar 460800   → model=0x41
  A1M8  ：python3 lidar_probe.py /dev/rplidar 115200   → 韌體 1.29
⚠ 波特率給錯＝完全無回應，症狀跟「光達壞掉」一模一樣（2026-07-20 踩過）。
"""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ydlidar"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 460800

STOP = b"\xA5\x25"
GET_INFO = b"\xA5\x50"
GET_HEALTH = b"\xA5\x52"


def ask(ser, cmd, want_payload):
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.15)
    desc = ser.read(7)
    if len(desc) < 7 or desc[0] != 0xA5 or desc[1] != 0x5A:
        return None, desc
    payload = ser.read(want_payload)
    return payload, desc


def main():
    print(f"port={PORT} baud={BAUD}")
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    # 先停掃描，確保它不是停在連續吐資料的狀態
    ser.write(STOP)
    ser.flush()
    time.sleep(0.05)
    ser.reset_input_buffer()

    info, desc = ask(ser, GET_INFO, 20)
    if info is None or len(info) < 20:
        print(f"GET_INFO 失敗，descriptor={desc.hex() if desc else 'none'}")
        print("→ 波特率可能不對，或本體沒回應")
        ser.close()
        return 1
    model = info[0]
    fw_minor, fw_major = info[1], info[2]
    hw = info[3]
    sn = info[4:20][::-1].hex().upper()
    print(f"型號代碼 model=0x{model:02X} ({model})")
    print(f"韌體版本 firmware={fw_major}.{fw_minor}")
    print(f"硬體版本 hardware={hw}")
    print(f"序號 SN={sn}")

    health, desc = ask(ser, GET_HEALTH, 3)
    if health and len(health) >= 3:
        status = health[0]
        errcode = health[1] | (health[2] << 8)
        名稱 = {0: "Good", 1: "Warning", 2: "Error"}.get(status, "Unknown")
        print(f"健康狀態 health={status} ({名稱}) error_code=0x{errcode:04X}")
    else:
        print("GET_HEALTH 無回應")

    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
