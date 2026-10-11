#!/bin/bash
# V2 雙車模擬一條龍：清殘留 -> 起 sim -> 評分 64s -> 收屍。輸出到 /tmp/score.log。
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

clean() {
  # ⚠ "ros2 bag play" 的程序 cmdline 不含 rosbag2_player（那是節點名）——
  #   要用 "[b]ag play" 才殺得到（2026-07-11 殘留回放污染 V2 評分的教訓）
  for p in "[k]inematic_sim" "[f]igure8_driver" "[v]2v_relay" "follow[e]r" "score[_]follow" "[b]ag play" "[s]im_bag_leader" "[s]im_two_car"; do
    pkill -9 -f "$p" 2>/dev/null
  done
}

: > /tmp/score.log
clean
sleep 2

# 起雙車 sim（背景）
setsid nohup bash -lc "source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; exec ros2 launch stm32_bridge sim_two_car.launch.py" >/tmp/sim.log 2>&1 </dev/null &
sleep 4

# 對帳：leader topic 的發布者必須恰為 1（殘留節點/回放雙發布是已知地雷）
PUBS=$(ros2 topic info /car1/odometry/filtered 2>/dev/null | grep -oP "Publisher count: \K\d+")
if [ "$PUBS" != "1" ]; then
  echo "ABORT: /car1/odometry/filtered 發布者=$PUBS（應為 1），環境不乾淨" >>/tmp/score.log
  clean; echo "DONE" >>/tmp/score.log; exit 1
fi

# 前景跑評分器（涵蓋整個 figure8）
ros2 run stm32_bridge score_follow --csv /tmp/s.csv --duration 64 >>/tmp/score.log 2>&1
echo "SCORE_EXIT=$?" >>/tmp/score.log

clean
echo "DONE" >>/tmp/score.log
