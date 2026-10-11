#!/bin/bash
# wash_bag.sh <輸入bag目錄> <輸出bag目錄>
# 離線洗 bag：只回放原始感測 (/odom /imu/data)，用新版 ekf.yaml 重跑 EKF
# （只吃角速度、從原點零航向起算），錄下乾淨的 /odometry/filtered。
# 產物 = yaw 無死區失真 + 起點自動歸零的 leader 軌跡（V3 用）。
# （不用 set -u：ROS setup.bash 內部引用未定義變數，會被它誤殺。）
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

BAG="$1"; OUT="$2"
EKF_PARAMS=~/ros2_ws/src/stm32_bridge/config/ekf.yaml
RATE=3.0

if [ -d "$OUT" ]; then
  echo "ERROR: 輸出目錄已存在，不覆蓋：$OUT"; exit 1
fi

clean_ekf() { pkill -9 -f "[e]kf_node" 2>/dev/null; }

clean_ekf
sleep 1

# 1) EKF（sim time，等 /clock）
setsid nohup ros2 run robot_localization ekf_node --ros-args \
  --params-file "$EKF_PARAMS" -p use_sim_time:=true \
  >/tmp/wash_ekf.log 2>&1 </dev/null &
sleep 3

# 2) 錄新的 /odometry/filtered
setsid nohup ros2 bag record -s mcap -o "$OUT" /odometry/filtered \
  >/tmp/wash_rec.log 2>&1 </dev/null &
sleep 2
REC_PID=$(pgrep -f "[r]os2 bag record -s mcap -o $OUT" | head -1)

# 3) 回放原始感測（前景，播完即返回）。--clock 100：sim 時鐘發 100Hz——
#    EKF 計時器跟著 /clock tick 走，預設 40Hz 在 rate 3 下輸出會被壓到 ~13Hz。
ros2 bag play "$BAG" --clock 100 --rate $RATE --topics /odom /imu/data \
  >/tmp/wash_play.log 2>&1
sleep 3

# 4) TERM 收錄器（SIGTERM 才會 finalize 出 metadata.yaml，runbook 已知坑）
[ -n "${REC_PID:-}" ] && kill -TERM "$REC_PID" 2>/dev/null
for i in $(seq 1 20); do
  [ -f "$OUT/metadata.yaml" ] && break
  sleep 1
done
clean_ekf

if [ -f "$OUT/metadata.yaml" ]; then
  echo "WASH_OK $OUT"
else
  echo "WASH_FAIL $OUT (metadata.yaml 沒出來，看 /tmp/wash_rec.log)"; exit 1
fi
