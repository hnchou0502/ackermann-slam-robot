#!/usr/bin/env python3
"""RPLIDAR A1M8 復位工具（car2）。

用途：rplidar 驅動被硬殺（kill -9）後，光達會停留在「連續吐掃描資料」模式，
下一次驅動啟動時 GET_INFO 對不上而 RESULT_OPERATION_TIMEOUT。
重啟驅動前先跑本工具送 STOP＋RESET，讓光達回到乾淨狀態（2026-07-17 實戰兩次）。

另附 --hold：壓停馬達並抓著埠不放（A1M8 硬體預設「上電就轉」，
只有程式持有埠並拉住 DTR 才能讓馬達停）。鎖螺絲/收工時用，Ctrl-C 或 kill 結束。

用法（在 car2 Pi 上）：
    python3 rplidar_reset.py            # STOP+RESET 後退出（重啟驅動前跑這個）
    python3 rplidar_reset.py --hold     # 停馬達並常駐壓住（收工時用）
"""
import argparse
import sys
import time

import serial

PORT = '/dev/rplidar'
BAUD = 115200  # A1M8 固定 115200（256000 是 A3，實測不回話）


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--hold', action='store_true',
                    help='壓停馬達並抓著埠常駐（預設：復位後退出）')
    args = ap.parse_args()

    s = serial.Serial(args.port, BAUD, timeout=1)
    s.write(b'\xA5\x25')          # STOP：停止掃描
    time.sleep(0.3)
    if args.hold:
        s.dtr = True              # setDTR＝馬達停（A1 轉接板 DTR 反相）
        print(f'{args.port}: 馬達已壓停，抓著埠中（kill 我或 Ctrl-C 結束）', flush=True)
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            pass
    else:
        s.write(b'\xA5\x40')      # RESET：光達軟重開，清掉殘留的串流狀態
        time.sleep(2)
        s.reset_input_buffer()
        print(f'{args.port}: STOP+RESET 完成，可重啟驅動')
    s.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
