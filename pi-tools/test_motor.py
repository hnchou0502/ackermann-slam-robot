"""
⚠ 已過期 (2026-07-08 標記)：Pi4 時代的馬達/FBK 手動測試腳本，與現役系統有兩處不符——
   - SERIAL_PORT=/dev/ttyUSB0 ≠ 現役 udev 固定埠 /dev/stm32（BAUD_RATE 115200 本身正確）
   - FBK 解析僅 4 資料欄（$FBK,vL,vR,pwmL,pwmR,#）；現役韌體已擴充為 7 欄
     （多 gyroZ, yaw），照此檔解析會靜默忽略 IMU 兩欄。
   現役串列協定以 ros2_ws/src/stm32_bridge/stm32_bridge/bridge_node.py 為準。
   保留此檔僅作歷史參考。
"""
import serial
import time
import threading
import sys

# 設定 UART (請修改 '/dev/ttyUSB0' 為你的實際 Port)
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

ser = None
running = True

def connect_serial():
    global ser
    while running:
        try:
            if ser is None or not ser.is_open:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                print(f"Successfully connected to {SERIAL_PORT}")
                return True
        except Exception as e:
            print(f"Connection failed: {e}, retrying in 2s...")
            time.sleep(2)
    return False

def read_serial():
    global ser
    while running:
        if ser and ser.is_open:
            try:
                # 加強保護：檢查 in_waiting 也可能出錯
                if ser.in_waiting > 0:
                    try:
                        # 讀取一行並解碼
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        # 檢查字串是否為空
                        if not line:
                            continue

                        # 檢查是否為回傳數據格式 "$FBK,..."
                        if line.startswith("$FBK"):
                            parts = line.split(',')
                            # 解析格式: $FBK,L_SPD,R_SPD,L_PWM,R_PWM,#
                            # 確保分割後的長度足夠且不是空字串
                            if len(parts) >= 6 and parts[1] and parts[2]:
                                try:
                                    l_spd = float(parts[1])
                                    r_spd = float(parts[2])
                                    l_pwm = int(parts[3]) if parts[3] else 0
                                    r_pwm = int(parts[4]) if parts[4] else 0
                                    print(f"[STM32] L_Speed: {l_spd:.3f}, R_Speed: {r_spd:.3f} | PWM: {l_pwm}, {r_pwm}")
                                except ValueError:
                                    pass
                    except Exception as e:
                        print(f"Read/Decode Error: {e}")
            except OSError as e:
                print(f"Serial Port Error (Device disconnected?): {e}")
                ser.close()
                ser = None
                # 嘗試重新連線
                connect_serial()
            except Exception as e:
                print(f"Unexpected Error in read_serial: {e}")
                time.sleep(0.1)
        else:
            time.sleep(1)

# 先嘗試連線
if not connect_serial():
    print("Failed to connect. Exiting.")
    sys.exit(1)

# 啟動接收執行緒
t = threading.Thread(target=read_serial)
t.daemon = True
t.start()

try:
    # 測試流程
    print("Sending: GO 0.3 m/s")
    if ser and ser.is_open: ser.write(b"$CMD,0.3,0.0,#\n")
    time.sleep(3)
    
    print("Sending: STOP")
    if ser and ser.is_open: ser.write(b"$CMD,0.0,0.0,#\n")
    time.sleep(1)
    
    print("Sending: BACK -0.3 m/s")
    if ser and ser.is_open: ser.write(b"$CMD,-0.3,0.0,#\n")
    time.sleep(3)
    
    print("Sending: STOP")
    if ser and ser.is_open: ser.write(b"$CMD,0.0,0.0,#\n")

except KeyboardInterrupt:
    print("Exiting...")
finally:
    running = False
    if ser and ser.is_open:
        try:
            ser.write(b"$CMD,0.0,0.0,#\n") # 安全停機
            ser.close()
        except:
            pass
    print("Closed.")
