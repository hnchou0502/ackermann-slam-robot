import serial
import time

# 設定目前的 Port
PORT = '/dev/ttyUSB0'
BAUD = 115200

print(f"準備開啟 {PORT}，速率 {BAUD}...")

try:
    # 開啟 Serial
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"成功開啟 {PORT}！正在監聽數據...")
    print("如果不對勁，請按 Ctrl+C 離開")
    print("-" * 30)
    
    while True:
        if ser.in_waiting > 0:
            # 讀取並解碼
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"收到: {line}")
            except Exception as e:
                print(f"解碼錯誤: {e}")
        else:
            time.sleep(0.01)

except Exception as e:
    print(f"\n無法開啟 Serial: {e}")
    print("檢查建議：sudo chmod 666 /dev/ttyUSB0")
except KeyboardInterrupt:
    print("\n測試結束")
