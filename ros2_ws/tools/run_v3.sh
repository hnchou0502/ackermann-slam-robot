#!/bin/bash
# V3 半實車一條龍：洗過的 bag 當 leader -> follower(模擬) 跟隨 -> 獨立評分。
# 輸出 /tmp/v3_score.log；產物 /tmp/v3_score.csv。
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

BAG=${1:-$HOME/bags/washed/map3_washed}
DURATION=${2:-900}

clean() {
  for p in "[k]inematic_sim" "[v]2v_relay" "follow[e]r" "score[_]follow" "[r]osbag2_player" "[s]im_bag_leader"; do
    pkill -9 -f "$p" 2>/dev/null
  done
}

: > /tmp/v3_score.log
clean
sleep 2

# 起 launch（relay + car2 + bag play@+3s），背景
setsid nohup bash -lc "source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; exec ros2 launch stm32_bridge sim_bag_leader.launch.py bag:=$BAG" >/tmp/v3_launch.log 2>&1 </dev/null &
sleep 8   # relay/follower 就位 + bag 已開播，讓評分器一起跑就有資料（deadline 餘量足）

# 評分（前景，直到 duration 或 wall-clock 兜底）
ros2 run stm32_bridge score_follow \
  --leader-topic /car1/v2v/state --follower-topic /car2/odometry/filtered \
  --csv /tmp/v3_score.csv --duration "$DURATION" >>/tmp/v3_score.log 2>&1
echo "SCORE_EXIT=$?" >>/tmp/v3_score.log

clean
echo "V3_DONE" >>/tmp/v3_score.log
