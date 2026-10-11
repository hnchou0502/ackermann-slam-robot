#!/usr/bin/env python3
import rospy
import serial
import math
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry

# --- 設定 ---
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200

# [幾何參數]
WHEEL_BASE = 0.235  # 23.5cm
TRACK_WIDTH = 0.14  # 14cm

class STM32Driver:
    def __init__(self):
        rospy.init_node('stm32_bridge_node')
        
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            rospy.loginfo(f"Success: Connected to {SERIAL_PORT}")
        except Exception as e:
            rospy.logerr(f"Serial Error: {e}")
            exit(1)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.rate = rospy.Rate(50) 
        
        rospy.loginfo("Bridge Started. Waiting for data...")
        
        while not rospy.is_shutdown():
            self.read_serial()
            self.rate.sleep()

    def cmd_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        
        # [阿克曼保護]
        if abs(v) < 0.01 and abs(w) > 0.01:
            w = 0.0

        # [v66.0 修正] 發送整數 (mm/s, mrad/s) 給 STM32
        # 例如: 0.5 m/s -> 500
        v_int = int(v * 1000)
        w_int = int(w * 1000)

        cmd_str = f"$CMD,{v_int},{w_int},#\n"
        
        # [DEBUG] 印出傳送內容確保正確
        # rospy.loginfo(f"TX: {cmd_str.strip()}")

        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except:
            pass

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                rospy.loginfo(f"Raw: {line}")

                # 解析 v66.0 協議: $FBK,vL_mm,vR_mm,pwmL,pwmR,#
                if "$FBK" in line and line.endswith("#"):
                    clean_line = line[line.find("$FBK"):]
                    parts = clean_line.split(',')
                    
                    if len(parts) >= 5:
                        try:
                            # 接收整數 mm/s -> 轉回 float m/s
                            v_l = float(parts[1]) / 1000.0
                            v_r = float(parts[2]) / 1000.0
                            self.update_odometry(v_l, v_r)
                        except ValueError:
                            pass
        except Exception as e:
            pass

    def update_odometry(self, v_l, v_r):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        v_linear = (v_r + v_l) / 2.0
        v_angular = (v_r - v_l) / TRACK_WIDTH 

        delta_x = v_linear * math.cos(self.th) * dt
        delta_y = v_linear * math.sin(self.th) * dt
        delta_th = v_angular * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position = Point(self.x, self.y, 0.)
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        STM32Driver()
    except rospy.ROSInterruptException:
        pass
