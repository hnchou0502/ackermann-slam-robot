#!/bin/bash
# EX4／RM3 自主探索實跑：一趟的起跑、看進度、急停、收尾。在 Pi 上執行。
#
# 用法：
#   ex4_run.sh start <名稱> [--timeout 600]   # 起 explore.launch（唯一合法入口）＋foxglove＋bag
#   ex4_run.sh status <名稱>                   # 看探索進度（explorer 最近幾行＋目前覆蓋率）
#   ex4_run.sh stop <名稱>                     # 急停：SIGINT 給 supervisor 的整個行程群組
#   ex4_run.sh finish <名稱>                   # 存姿態圖＋存地圖＋算覆蓋率＋查限速＋收乾淨
#   ex4_run.sh clean                           # 收掉所有殘留節點
#
# 產物一律進 ~/ex4/<名稱>/：explore.log、bag/、<名稱>.pgm/.yaml、<名稱>.posegraph/.data、verdict.txt
#
# 為什麼要有這支腳本（2026-09-24）：EX4 前三趟（2026-08-30）是手打指令，
#   ①pkill 的模式字串出現在 ssh 命令列上會殺掉自己的連線（一天踩四次）
#   ②覆蓋率／限速是事後手算，容易漏項。
#   這裡把兩者都收進檔案，ssh 命令列只剩「ex4_run.sh <子指令> <名稱>」。
#
# ⚠ set -u 必須放在 source ROS setup 之後——setup.bash 自己會讀未定義變數。

set -o pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
set -u

TOOLS="$HOME/ros2_ws/tools"
BASELINE_M2=24.65     # 遙控圖 c1_living1 的已知空地面積（EX4 覆蓋率分母）
PASS_RATIO=0.70       # EX4 判準：覆蓋率 ≥70%

CMD="${1:-}"
NAME="${2:-}"
if [ "$CMD" = "clean" ]; then
  NAME="-"; shift 1
else
  [ -n "$CMD" ] && [ -n "$NAME" ] || {
    echo "用法：ex4_run.sh start|status|stop|finish <名稱>   或   ex4_run.sh clean"
    exit 2; }
  shift 2
fi
RUNDIR="$HOME/ex4/$NAME"

# 逐個節點執行檔點名收掉（清單比 rm2_pass.sh 多了 Nav2 全家）。
# ⚠ 殺 launch 母行程「不會」帶走被 init 收養的子節點，所以要逐個點名。
# ⚠ 執行檔叫 `bridge` 不是 `bridge_node`（2026-08-30 踩過）。
cleanup_nodes() {
  local pats=("ros2 launch stm32_bridge explore" "ros2 launch stm32_bridge bringup"
              frontier_explorer controller_server planner_server behavior_server
              bt_navigator smoother_server velocity_smoother waypoint_follower
              lifecycle_manager collision_monitor slam_toolbox_node foxglove_bridge
              "bag record" rosbag2 "stm32_bridge/bridge" ekf_node sllidar_node
              static_transform_publisher)
  local sig p
  for sig in TERM KILL; do
    for p in "${pats[@]}"; do pkill -"$sig" -f "$p" >/dev/null 2>&1; done
    sleep 3
  done
}

# 覆蓋率＝地圖中「確定是空地」的格子面積 / 遙控圖的空地面積。
# 與 2026-08-30 三趟的算法一致（pgm 值 >250 的格子 × 0.05²；c1_living1 = 24.65 m²）。
# Pi 上不一定有 PIL，所以手解 P5 標頭。
coverage_of() {
  python3 - "$1" "$BASELINE_M2" <<'EOF'
import sys
b = open(sys.argv[1], 'rb').read()
parts, i = [], 0
while len(parts) < 4:
    while b[i:i+1].isspace():
        i += 1
    if b[i:i+1] == b'#':
        while b[i:i+1] != b'\n':
            i += 1
        continue
    j = i
    while not b[j:j+1].isspace():
        j += 1
    parts.append(b[i:j]); i = j
free = sum(1 for v in b[i+1:] if v > 250) * 0.05 * 0.05
print('%.2f %.3f' % (free, free / float(sys.argv[2])))
EOF
}

# ---------------- clean ----------------
if [ "$CMD" = "clean" ]; then
  cleanup_nodes
  echo "殘留 ROS 行程："
  ps -eo pid,args | grep -E "[s]lam_toolbox|stm32_bridge/[b]ridge|[e]kf_node|[s]llidar|[f]oxglove|[r]osbag2|[f]rontier_explorer|[c]ontroller_server|[b]t_navigator" || echo "  (乾淨)"
  BUSY=$(sudo fuser /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null | tr -d " ")
  [ -n "$BUSY" ] && echo "⚠ 序列埠仍被佔用：$BUSY" || echo "序列埠：沒人佔用"
  exit 0
fi

# ---------------- start ----------------
if [ "$CMD" = "start" ]; then
  TIMEOUT=600.0
  while [ $# -gt 0 ]; do
    case "$1" in
      --timeout) TIMEOUT="$2"; shift 2 ;;
      *) echo "未知參數：$1"; exit 2 ;;
    esac
  done

  cleanup_nodes
  BUSY=$(sudo fuser /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null | tr -d " ")
  if [ -n "$BUSY" ]; then
    echo "❌ 序列埠仍被佔用（PID: $BUSY），拒絕開跑——兩套節點會搶埠"
    exit 3
  fi
  # 目的地已存在＝拒跑：bag record 遇既有資料夾會靜默失敗（2026-08-30 踩過）
  if [ -e "$RUNDIR" ]; then
    echo "❌ $RUNDIR 已存在。換個名稱。"; exit 5
  fi
  mkdir -p "$RUNDIR"

  echo "▶ 起 explore.launch（supervisor；bringup＋Nav2(slam)＋explorer，總時限 ${TIMEOUT}s）"
  # setsid ⇒ launch 是自己行程群組的組長，PGID＝PID；急停時對整組送 SIGINT（等同 Ctrl-C）
  setsid nohup ros2 launch stm32_bridge explore.launch.py explore_timeout_s:="$TIMEOUT" \
    > "$RUNDIR/explore.log" 2>&1 </dev/null &
  echo $! > "$RUNDIR/supervisor.pgid"
  disown

  echo "▶ 起 foxglove_bridge（port 8765，供筆電看圖）"
  setsid nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 \
    > "$RUNDIR/foxglove.log" 2>&1 </dev/null & disown

  sleep 20
  echo "▶ 起 bag 錄製（/odom＝FBK 實際輪速，限速驗收看它；/map 供事後重算覆蓋率）"
  setsid nohup ros2 bag record -s mcap -o "$RUNDIR/bag" \
    /odom /imu/data /odometry/filtered /scan /tf /tf_static /cmd_vel /map \
    > "$RUNDIR/rec.log" 2>&1 </dev/null & disown
  sleep 5
  if ! ls "$RUNDIR"/bag/*.mcap >/dev/null 2>&1; then
    echo "❌ bag 錄製沒起來（見 $RUNDIR/rec.log）——急停整串"
    kill -INT -- -"$(cat "$RUNDIR/supervisor.pgid")" 2>/dev/null
    sleep 5; cleanup_nodes
    tail -3 "$RUNDIR/rec.log"; exit 5
  fi

  echo ""
  echo "===== 開跑後對帳 ====="
  RC=0
  # ⚠ 剛起的 ros2 daemon 還沒掃到節點時會回報 0／查不到——2026-09-24 run4 就是這樣
  #   假警報、白白急停一趟。所以重試到 30 秒，恰好 1 個才算過（>1＝殘留節點污染）。
  for t in /scan /odom /map; do
    N=""
    for _ in 1 2 3 4 5 6; do
      N=$(timeout 10 ros2 topic info "$t" 2>/dev/null | awk "/Publisher count/{print \$3}")
      [ "${N:-0}" = "1" ] && break
      [ "${N:-0}" -gt 1 ] 2>/dev/null && break
      sleep 5
    done
    printf "%-10s 發布者 %s " "$t" "${N:-?}"
    if [ "${N:-0}" = "1" ]; then echo "✅"; else echo "❌（應為 1）"; RC=1; fi
  done
  if ! kill -0 "$(cat "$RUNDIR/supervisor.pgid")" 2>/dev/null; then
    echo "❌ supervisor 已經不在了"; RC=1
  fi
  echo "--- explorer 最近訊息 ---"
  grep -E "frontier_explorer|EXPLORE" "$RUNDIR/explore.log" | tail -5
  if [ "$RC" != "0" ]; then
    echo "❌ 對帳失敗——急停整串"
    kill -INT -- -"$(cat "$RUNDIR/supervisor.pgid")" 2>/dev/null
    sleep 5; cleanup_nodes                # bag／foxglove 不在 supervisor 群組裡，要另外收
    exit 4
  fi
  echo "✅ 探索已開始。急停：ex4_run.sh stop $NAME（或拔 STM32 USB 線）"
  exit 0
fi

# ---------------- status ----------------
if [ "$CMD" = "status" ]; then
  [ -d "$RUNDIR" ] || { echo "❌ 找不到 $RUNDIR"; exit 2; }
  if kill -0 "$(cat "$RUNDIR/supervisor.pgid")" 2>/dev/null; then
    echo "supervisor：活著"
  else
    echo "supervisor：⚠ 已結束"
  fi
  echo "--- explorer 最近訊息 ---"
  grep -E "目標 #|EXPLORATION DONE|黑名單|blacklist|NOT_READY|收穫|FATAL|ERROR" "$RUNDIR/explore.log" | tail -8
  TMP=$(mktemp -d)
  if timeout 25 ros2 run nav2_map_server map_saver_cli -f "$TMP/now" \
       --ros-args -p save_map_timeout:=15.0 >/dev/null 2>&1 && [ -f "$TMP/now.pgm" ]; then
    read -r M2 R <<< "$(coverage_of "$TMP/now.pgm")"
    echo "目前覆蓋：${M2} m²（$(python3 -c "print('%.1f' % ($R*100))")%，判準 ≥70%）"
  else
    echo "（目前抓不到 /map）"
  fi
  rm -rf "$TMP"
  exit 0
fi

# ---------------- stop（急停） ----------------
if [ "$CMD" = "stop" ]; then
  [ -f "$RUNDIR/supervisor.pgid" ] || { echo "❌ 找不到 supervisor.pgid"; exit 2; }
  PG=$(cat "$RUNDIR/supervisor.pgid")
  echo "▶ SIGINT → 行程群組 $PG（等同在 supervisor 終端按 Ctrl-C）"
  kill -INT -- -"$PG" 2>/dev/null
  sleep 5
  grep -E "STOP_CONFIRMED|FATAL" "$RUNDIR/explore.log" | tail -3
  cleanup_nodes                           # bag／foxglove 不在 supervisor 群組裡
  echo "⚠ 地圖沒有存。要存的話不要用 stop，用 finish。"
  exit 0
fi

# ---------------- finish ----------------
if [ "$CMD" = "finish" ]; then
  [ -d "$RUNDIR" ] || { echo "❌ 找不到 $RUNDIR"; exit 2; }
  PG=$(cat "$RUNDIR/supervisor.pgid")

  # 先讓車停：explorer 收工後本來就會 _stop_car，但人為提早收尾時車可能還在走。
  # 存圖需要 slam_toolbox 還活著，所以不能先關整串——只取消 Nav2 目標、壓零速。
  echo "▶ 取消導航目標＋送零速（車停下來再存圖）"
  timeout 10 ros2 service call /navigate_to_pose/_action/cancel_goal \
    action_msgs/srv/CancelGoal "{}" >/dev/null 2>&1
  timeout 5 ros2 topic pub --times 5 --rate 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1

  echo "▶ 停 bag（TERM 才會 finalize metadata）"
  pkill -TERM -f "bag record" >/dev/null 2>&1
  sleep 4

  echo "▶ 存姿態圖（.posegraph + .data，RM3 續建要用）"
  timeout 40 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$RUNDIR/$NAME'}" > "$RUNDIR/serialize.log" 2>&1
  [ -f "$RUNDIR/$NAME.posegraph" ] && echo "  ✅ $NAME.posegraph" || echo "  ⚠ 沒產出 posegraph（見 serialize.log）"

  echo "▶ 存地圖（.pgm + .yaml）"
  ros2 run nav2_map_server map_saver_cli -f "$RUNDIR/$NAME" \
    --ros-args -p save_map_timeout:=30.0 > "$RUNDIR/save.log" 2>&1
  [ -f "$RUNDIR/$NAME.pgm" ] || { echo "❌ 沒產出 $NAME.pgm，見 save.log"; }

  echo "▶ 收乾淨（SIGINT 整組 → 殘留點名清）"
  kill -INT -- -"$PG" 2>/dev/null
  sleep 8
  cleanup_nodes

  {
    echo "===== EX4 驗收：$NAME ====="
    echo "--- 收工原因 ---"
    grep -E "EXPLORATION DONE" "$RUNDIR/explore.log" | tail -1 || echo "(explorer 沒有自己收工——人為 finish)"
    echo "目標總數：$(grep -c '目標 #' "$RUNDIR/explore.log")"
    if [ -f "$RUNDIR/$NAME.pgm" ]; then
      read -r M2 R <<< "$(coverage_of "$RUNDIR/$NAME.pgm")"
      if python3 -c "import sys; sys.exit(0 if $R >= $PASS_RATIO else 1)"; then V="✅ PASS"; else V="❌ FAIL"; fi
      echo "覆蓋：${M2} m² / ${BASELINE_M2} m² = $(python3 -c "print('%.1f' % ($R*100))")%（判準 ≥70%）$V"
    fi
    echo "--- 限速（FBK 實際輪速才算數，EKF 不算）---"
    python3 "$TOOLS/check_speed_limit.py" "$RUNDIR/bag" 2>&1 | tail -12
  } | tee "$RUNDIR/verdict.txt"
  echo "殘留行程（應為 0）：$(ps -eo args | grep -cE "[f]rontier_explorer|[s]lam_toolbox_node|[c]ontroller_server")"
  exit 0
fi

echo "未知指令：$CMD"; exit 2
