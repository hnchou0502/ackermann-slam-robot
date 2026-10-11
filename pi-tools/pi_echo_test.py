#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
【專案藍圖 - 第 1 週】Echo 測試腳本
版本：1.1 (已修正埠口名稱)

⚠ 已過期 (2026-07-08 標記)：此為 Pi4 時代的第 1 週連通性測試，設定與現役系統不符——
   - BAUD_RATE=9600 ≠ 現役韌體 UART1 的 115200（照此值根本收不到 $FBK）
   - SERIAL_PORT=/dev/ttyUSB0 ≠ 現役 udev 固定埠 /dev/stm32
   現役串列協定與埠設定以 ros2_ws/src/stm32_bridge/stm32_bridge/bridge_node.py 為準。
   保留此檔僅作歷史參考；除錯請勿照抄本檔的埠/鮑率。
"""

import serial
import time
import sys

# --- 設定 ---
# 在 Pi 4 + Ubuntu 20.04 上，硬體 UART 通常是 /dev/ttyS0
# 如果 /dev/ttyS0 不行，可以試試看 /dev/ttyAMA0
SERIAL_PORT = "/dev/ttyUSB0" 
BAUD_RATE = 9600
TIMEOUT_SECONDS = 5.0 # 等待 5 秒
CHAR_TO_SEND = 'A'
# --- 結束設定 ---

def print_failure_help():
    """顯示詳細的故障排除指南"""
    print("\n--- 故障排除指南 ---")
    print("1. 【接線】: 90% 的問題出在這裡！請「徹底」檢查：")
    print("   - [GND] Pi Pin 6 <-> STM32 Pin ⑮(SBUS)-G (最上面)")
    print("   - [TX]  Pi Pin 8 <-> STM32 Pin ⑮(SBUS)-S (最下面) (Pi TX -> STM32 RX)")
    print("   - [RX]  Pi Pin 10 <-> STM32 Pin ⑭(RGB)-S (最下面) (Pi RX -> STM32 TX)")
    print("\n2. 【STM32】:")
    print("   - 鮑率 (Baud Rate) 是否設為 9600？")
    print("   - `USART1 global interrupt` 真的啟用了嗎？")
    print("   - 程式碼是否已燒錄？是否已重新上電？")
    print("\n3. 【樹莓派】:")
    print("   - 您是否已遵照 README，編輯 /boot/firmware/cmdline.txt 和 config.txt 並「重新開機」？")
    print(f"   - 埠口名稱 {SERIAL_PORT} 是否正確？")

def main():
    """主執行函式"""
    print(f"正在對序列埠 {SERIAL_PORT} (鮑率 {BAUD_RATE}) 進行 Echo 測試...")
    
    # 檢查埠口是否存在
    import os
    if not os.path.exists(SERIAL_PORT):
        print(f"錯誤：找不到序列埠 {SERIAL_PORT}！")
        print("您是否忘記執行「步驟三」來啟用樹莓派的 UART 硬體了？")
        print("請檢查 README.md 中的步驟 3.1 並重新開機。")
        sys.exit(1)

    ser = None # 初始化
    try:
        # 打開序列埠
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=TIMEOUT_SECONDS
        )

        print(f"發送字元: {CHAR_TO_SEND}")
        
        # 發送一個位元組 (需要 encode 成 bytes)
        ser.write(CHAR_TO_SEND.encode('utf-8'))
        
        print("正在等待 STM32 的回音...")

        # 讀取一個位元組
        response_byte = ser.read(1)

        # 檢查是否有收到回覆
        if not response_byte:
            print(f"\n測試失敗：超時 ({TIMEOUT_SECONDS}秒)，未收到任何回覆。")
            print_failure_help()
            sys.exit(1)

        # 解碼
        response_char = response_byte.decode('utf-8')

        print(f"成功收到回音: {response_char}")

        # 驗證回音是否正確
        if response_char == CHAR_TO_SEND:
            print("\n測試成功！通訊正常！🎉")
        else:
            print(f"\n測試失敗：回音不符！(收到 {response_char} 但應為 {CHAR_TO_SEND})")
            print("請檢查 STM32 上的程式邏輯是否正確。")

    except serial.SerialException as e:
        print(f"\n序列埠錯誤: {e}")
        print(f"請檢查 {SERIAL_PORT} 是否被其他程式佔用，或您是否有權限存取。")
        print("嘗試使用 'sudo python3 pi_echo_test.py' 執行看看。")
    
    except Exception as e:
        print(f"\n發生未預期的錯誤: {e}")
    
    finally:
        if ser and ser.is_open:
            ser.close()
            print(f"序列埠 {SERIAL_PORT} 已關閉。")

if __name__ == "__main__":
    main()

