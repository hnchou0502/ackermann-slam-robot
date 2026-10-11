#!/usr/bin/env python3
"""STM32 <-> ROS2 橋接（讀回授 + 送指令，單一程式佔序列埠）。

讀取 STM32 透過自訂 UART 協定送出的回授封包。支援兩種格式（向後相容）：
    舊: $FBK,vL,vR,pwmL,pwmR,#
    新: $FBK,vL,vR,pwmL,pwmR,gyroZ,yaw,#   （2026-06-28 韌體起）
        vL/vR  = mm/s 整數
        gyroZ  = IMU yaw rate, deg/s ×1000
        yaw    = IMU 積分航向, deg ×1000

發布：
    /odom            (nav_msgs/Odometry)        純編碼器航位推算
    /imu/data        (sensor_msgs/Imu)          IMU 角速度 + 積分航向（僅新格式）
    TF: odom -> base_link                       publish_tf=True 時
訂閱（enable_cmd=True 時）：
    /cmd_vel         (geometry_msgs/Twist)      -> 送 $CMD,vx*1000,wz*1000,#

⚠ 安全：enable_cmd=True 時這支「能讓車動」。三重保險：
   (1) 速度夾限 max_linear/max_angular；(2) dead-man：超過 cmd_timeout 沒收到
   /cmd_vel 就送停車；(3) 韌體本身 500ms 看門狗。要純唯讀就 -p enable_cmd:=false。
   阿克曼限制：|vx| 太小但 |wz| 大時 wz 歸零（車不能原地旋轉）。

備註（給之後接 robot_localization EKF 的自己）：
  - /odom 故意維持純輪速（含會發散的輪速差航向），當 EKF 的一路輸入；
    /imu/data 提供另一路（yaw rate + 積分 yaw）。讓 EKF 去融合，不要在這支先融。
  - EKF 上線後改 publish_tf=False，odom->base_link 那個 TF 交給 EKF 發，免得兩邊搶。
"""
import math
import sys
import time

import serial

from stm32_bridge.protocol import (CONFIRM_FRAMES, SERIAL_DEAD_AFTER_S,
                                   STOPPED_SPEED_MPS, CmdShaper,
                                   advance_stop_confirmation,
                                   format_cmd, parse_fbk, serial_should_abort)

import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

DEG2RAD = math.pi / 180.0


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # --- 參數（可在 launch / 命令列用 --ros-args -p 覆蓋）---
        self.declare_parameter('port', '/dev/stm32')   # udev 固定命名 (見 /etc/udev/rules.d/99-slamcar.rules)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('track_width', 0.14)   # 輪距 W (m)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'base_link')   # IMU 剛性裝在車體；之後可改 imu_link + 靜態 TF
        self.declare_parameter('publish_tf', True)         # EKF 上線後設 False
        self.declare_parameter('enable_cmd', True)         # False = 純唯讀，絕不送 $CMD
        self.declare_parameter('max_linear', 0.5)          # vx 夾限 (m/s)
        self.declare_parameter('max_angular', 2.0)         # wz 夾限 (rad/s)
        self.declare_parameter('cmd_rate', 20.0)           # 重送 $CMD 頻率 (Hz)，餵看門狗
        self.declare_parameter('serial_dead_after', SERIAL_DEAD_AFTER_S)
        self.declare_parameter('cmd_timeout', 0.5)         # dead-man：多久沒收到 /cmd_vel 就停 (s)
        # 指令整形（EX4 限速，2026-09-24；見 protocol.CmdShaper）。0＝關閉＝與加入前相同。
        self.declare_parameter('reversal_dwell_s', 0.0)    # 換向前先停多久 (s)
        self.declare_parameter('max_linear_accel', 0.0)    # 起步加速度上限 (m/s²)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.enable_cmd = bool(self.get_parameter('enable_cmd').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.shaper = CmdShaper(float(self.get_parameter('reversal_dwell_s').value),
                                float(self.get_parameter('max_linear_accel').value))
        self.serial_dead_after = float(self.get_parameter('serial_dead_after').value)
        self._serial_bad_since = None      # 序列埠連續失敗的起始時刻（None＝正常）
        # None＝還沒收到任何 FBK。**不可用 0.0 當初值**：停車確認會把「還沒收到資料」
        # 誤當成「輪速已歸零」而立刻宣告成功（Codex R2-F1，2026-08-11）。
        self.last_wheel_speed = None
        self.last_fbk_mono = None          # 最後一筆 FBK 的接收時刻（time.monotonic）
        cmd_rate = float(self.get_parameter('cmd_rate').value)

        try:
            # timeout=0 -> 非阻塞讀取，由 timer 定期抽取
            self.ser = serial.Serial(port, baud, timeout=0.0)
        except Exception as e:
            self.get_logger().error(f'無法開啟序列埠 {port}: {e}')
            raise

        mode = '讀+送(能讓車動)' if self.enable_cmd else '只讀不送'
        self.get_logger().info(
            f'已連上 {port} @ {baud}，輪距={self.track_width} m，publish_tf={self.publish_tf}，'
            f'enable_cmd={self.enable_cmd}。模式={mode}')
        if self.shaper.enabled:
            self.get_logger().info(
                f'指令整形：換向先停 {self.shaper.dwell:.2f} s、'
                f'起步加速度上限 {self.shaper.max_accel:.2f} m/s²')

        # 航位推算狀態
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.buf = b''
        self.fbk_count = 0
        self.imu_seen = False   # 是否曾收到含 IMU 的新格式

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 固定協方差（只有 yaw 與 yaw rate 可信；其餘標成大數=不可信）
        self._orient_cov = [1e6, 0.0, 0.0,
                            0.0, 1e6, 0.0,
                            0.0, 0.0, 0.05]
        self._gyro_cov = [1e6, 0.0, 0.0,
                          0.0, 1e6, 0.0,
                          0.0, 0.0, 0.01]

        # 100 Hz 抽序列埠（FBK 本身 25 Hz，抽快一點避免堆積延遲）
        self.timer = self.create_timer(0.01, self.read_serial)

        # --- 指令半邊（收 /cmd_vel -> 送 $CMD）---
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()
        if self.enable_cmd:
            self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
            # 定頻重送（餵韌體 500ms 看門狗）+ dead-man 檢查
            self.cmd_timer = self.create_timer(1.0 / cmd_rate, self.cmd_timer_tick)

    def cmd_cb(self, msg: Twist):
        v = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        w = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        # 阿克曼：低速時不能原地轉，wz 歸零（與韌體一致）
        if abs(v) < 0.01 and abs(w) > 0.01:
            w = 0.0
        self.cmd_v = v
        self.cmd_w = w
        self.last_cmd_time = self.get_clock().now()

    def cmd_timer_tick(self):
        # dead-man：太久沒收到 /cmd_vel 就送停車
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self.cmd_v = 0.0
            self.cmd_w = 0.0
        # 整形只會讓指令變小、不延後停車（test_cmd_shaper.py 的 S1–S3）；
        # 關閉流程的停車（stop_and_confirm）直接呼叫 send_cmd，不經過這裡。
        now = self.get_clock().now().nanoseconds * 1e-9
        v, w = self.shaper.step(self.cmd_v, self.cmd_w, now)
        self.send_cmd(v, w)

    def send_cmd(self, v: float, w: float):
        try:
            self.ser.write(format_cmd(v, w).encode('ascii'))
            self._serial_ok()
        except Exception as e:
            self.get_logger().warn(f'寫序列埠錯誤: {e}')
            self._serial_failed()

    def _serial_ok(self):
        self._serial_bad_since = None

    def _serial_failed(self):
        """序列埠讀寫失敗。持續超過門檻就結束行程，讓 supervisor 關掉整串。

        理由見 protocol.serial_should_abort：只停車而不關閉系統，會留下
        「插回線就可能突然開動」的殭屍狀態。
        """
        now = self.get_clock().now().nanoseconds / 1e9
        if self._serial_bad_since is None:
            self._serial_bad_since = now
            return
        if serial_should_abort(self._serial_bad_since, now, self.serial_dead_after):
            self.get_logger().fatal(
                f'序列埠已連續失敗 {self.serial_dead_after:.1f} 秒（線被拔掉或裝置消失）'
                f' → 結束行程，交由 supervisor 關閉整串控制堆疊')
            os._exit(1)

    def poll_wheel_speed(self):
        """只讀序列埠解析 FBK，**回傳本次實際解析到的輪速清單**（可能為空）。

        回傳清單而不是只更新欄位（Codex R3-F1）：呼叫端要能分辨「這輪有沒有拿到
        新資料」，只看欄位值會讓同一筆資料被重複採計。

        **不發布任何 topic**。

        關閉流程專用（stop_and_confirm）。不能直接用 read_serial()——它會順帶
        publish /odom，而此時 rclpy context 已關閉，publish 會拋
        `RCLError: publisher's context is invalid`，把停車確認整個打斷
        （2026-08-11 首次重測就是這樣失敗，bridge 還因此 exit code 1）。
        """
        speeds = []
        try:
            data = self.ser.read(512)
        except Exception:
            return speeds
        if not data:
            return speeds
        self.buf += data
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            fbk = parse_fbk(line.decode('ascii', 'ignore').strip())
            if fbk:
                sp = max(abs(fbk['vl']), abs(fbk['vr']))
                self.last_wheel_speed = sp
                self.last_fbk_mono = time.monotonic()
                speeds.append(sp)
        return speeds

    def read_serial(self):
        try:
            data = self.ser.read(512)
        except Exception as e:
            self.get_logger().warn(f'讀取序列埠錯誤: {e}')
            self._serial_failed()
            return
        self._serial_ok()
        if not data:
            return
        self.buf += data
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            self.handle_line(line.decode('ascii', 'ignore').strip())
        if len(self.buf) > 4096:   # 防呆：壞資料堆積就清掉
            self.buf = b''

    def handle_line(self, line: str):
        fbk = parse_fbk(line)
        if fbk is None:
            return

        v_l = fbk['vl']
        v_r = fbk['vr']
        self.last_wheel_speed = max(abs(v_l), abs(v_r))   # 關閉時確認停車用
        self.last_fbk_mono = time.monotonic()
        stamp = self.update_odom(v_l, v_r)

        if fbk['gyro_z'] is not None and fbk['yaw'] is not None:
            self.imu_seen = True
            self.publish_imu(stamp, fbk['gyro_z'], fbk['yaw'])

        self.fbk_count += 1
        if self.fbk_count % 50 == 0:
            imu_tag = 'IMU✓' if self.imu_seen else 'IMU✗(舊韌體?)'
            self.get_logger().info(
                f'FBK#{self.fbk_count} [{imu_tag}]  vL={v_l:+.3f} vR={v_r:+.3f} m/s  '
                f'odom x={self.x:+.2f} y={self.y:+.2f} th={math.degrees(self.th):+.1f}°')

    def update_odom(self, v_l: float, v_r: float):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:   # 第一筆或異常間隔，不積分這一步
            dt = 0.0

        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.track_width

        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += w * dt

        q = yaw_to_quaternion(self.th)
        stamp = now.to_msg()

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)
        return stamp

    def publish_imu(self, stamp, gyro_z: float, yaw: float):
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation = yaw_to_quaternion(yaw)
        imu.orientation_covariance = self._orient_cov
        imu.angular_velocity.z = gyro_z
        imu.angular_velocity_covariance = self._gyro_cov
        # 韌體沒送線加速度 -> 標記不可用（協方差首元 -1）
        imu.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0]
        self.imu_pub.publish(imu)


STOP_CONFIRM_TIMEOUT_S = 1.0     # 關閉時最多花這麼久確認車停（超過就放手交給韌體 watchdog）


def stop_and_confirm(node, timeout=STOP_CONFIRM_TIMEOUT_S):
    """關閉前送停車，並**持續讀 FBK 直到確認輪速歸零**才放手。回傳歸零耗時或 None。

    為什麼不是送完零速就退出（Codex F3，2026-08-11）：
      bridge 是唯一能讓車動的節點，它一死 serial 就斷、只剩韌體 500 ms watchdog 兜底。
      多花最多 1 秒確認車真的停了再退出，比立刻死掉安全；
      而且這是**唯一能以同一時基量到「FBK 輪速歸零時刻」的位置**——
      bridge 死後就沒有人在讀 FBK 了，事後無從證明車何時停。

    ⚠ **只認 t0 之後解析到的新 FBK，且要連續 CONFIRM_FRAMES 筆低於門檻**
      （Codex R2-F1）：舊版用 `last_wheel_speed` 的當下值判斷，初值 0.0 或停車命令
      **之前**的舊資料都會讓它立刻宣告成功——那是「把舊資料當新量測」，
      跟先前「把推論當量測」是同一類錯誤。
      逾時仍未確認就照樣退出（讓 watchdog 接手），但明確標示是哪一種失敗：
      `STOP_UNCONFIRMED`（有新資料但沒歸零）或 `STOP_NO_FBK`（整段沒收到任何新資料）。
    """
    if not getattr(node, 'enable_cmd', False):
        return None
    t0 = time.monotonic()
    node.last_wheel_speed = None       # 丟棄注入前的舊值
    node.last_fbk_mono = None
    consecutive = 0
    seen_fresh = False
    last_speed = None
    while time.monotonic() - t0 < timeout:
        node.send_cmd(0.0, 0.0)
        speeds = node.poll_wheel_speed()      # 只有本輪真的解析到的才算數
        if speeds:
            seen_fresh = True
            last_speed = speeds[-1]
        consecutive, confirmed = advance_stop_confirmation(speeds, consecutive)
        if confirmed:
            dt = time.monotonic() - t0
            print('STOP_CONFIRMED: stop_time_s=%.3f （連續 %d 筆獨立新 FBK 輪速 '
                  '%.3f m/s < %.3f）'
                  % (dt, CONFIRM_FRAMES, last_speed, STOPPED_SPEED_MPS),
                  file=sys.stderr, flush=True)
            return dt
        time.sleep(0.02)

    if not seen_fresh:
        print('STOP_NO_FBK: %.1f 秒內未收到任何新的 FBK，無法判定車是否停下 → '
              '放手交由韌體 watchdog 停車' % timeout, file=sys.stderr, flush=True)
    else:
        print('STOP_UNCONFIRMED: %.1f 秒內輪速未連續歸零（最後 %.3f m/s）→ '
              '放手交由韌體 watchdog 停車' % (timeout, last_speed),
              file=sys.stderr, flush=True)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = Stm32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop_and_confirm(node)
        except Exception as e:      # 確認停車本身出錯不該蓋掉正常關閉
            print('STOP_UNCONFIRMED: 確認過程出錯 %s' % e, file=sys.stderr, flush=True)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
