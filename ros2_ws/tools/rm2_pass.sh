#!/bin/bash
# RM2／RM3 實車二遍續建：一趟建圖的起跑與收尾。在 Pi 上執行。
#
# 用法：
#   rm2_pass.sh start <名稱>                                    # 第一遍（從零建圖）
#   rm2_pass.sh start <名稱> --load <posegraph前綴> [--start-pose x,y,yaw]
#                            [--slam-node async|lifelong]       # 第二遍（載入舊圖續建）
#                            ⚠ 預設 async。lifelong 在大圖會把姿態圖裁到碎裂、
#                              跨遍約束歸零（2026-08-30 RM2 實測，證據見下方註解）。
#   rm2_pass.sh finish <名稱>                                   # 存姿態圖＋存地圖＋收乾淨
#
# 產物一律進 ~/rm2/<名稱>/：<名稱>.posegraph/.data、<名稱>.pgm/.yaml、graph.json、bag/
#
# ⚠ 這段刻意寫成「檔案」而不是打在 ssh 命令列上：pkill -f 會比對整條命令列，
#    ssh 遠端命令字串裡若含節點名，pkill 會連自己這條 ssh 一起殺（runbook 記載的坑）。
#
# ⚠ set -u 必須放在 source ROS setup 之後——setup.bash 自己會讀未定義變數。

set -o pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
set -u

CMD="${1:-}"
NAME="${2:-}"
if [ "$CMD" = "clean" ]; then
  NAME="-"; shift 1
else
  [ -n "$CMD" ] && [ -n "$NAME" ] || {
    echo "用法：rm2_pass.sh start|finish <名稱> [--load <pg> --start-pose x,y,yaw]"
    echo "      rm2_pass.sh clean            # 收掉所有殘留節點"
    exit 2; }
  shift 2
fi

# 逐個節點執行檔點名收掉。⚠ 這些是「安裝後的真實執行檔名」，用 ls install/.../lib/ 查得到；
#   猜名字會靜默失效（bridge_node vs bridge）。
cleanup_nodes() {
  local pats=(slam_toolbox_node foxglove_bridge "bag record" rosbag2 bringup.launch
              "stm32_bridge/bridge" ekf_node sllidar_node static_transform_publisher
              frontier_explorer)
  local sig
  for sig in TERM KILL; do
    local p
    for p in "${pats[@]}"; do pkill -"$sig" -f "$p" >/dev/null 2>&1; done
    sleep 3
  done
}

RUNDIR="$HOME/rm2/$NAME"
TOOL="$HOME/ros2_ws/tools/check_posegraph_continuity.py"

# ---------------- clean ----------------
# ⚠ 一定要透過這個子指令清理，**不要**把 pkill 的模式字串打在 ssh 命令列上：
#   pkill -f 會比對整條命令列，ssh 遠端命令裡出現那些字串時會連自己的連線一起殺
#   （2026-08-30 一天內踩三次）。寫在腳本檔裡，ssh 命令列只剩 "rm2_pass.sh clean"。
if [ "$CMD" = "clean" ]; then
  cleanup_nodes
  echo "殘留 ROS 行程："
  ps -eo pid,args | grep -E "[s]lam_toolbox|stm32_bridge/[b]ridge|[e]kf_node|[s]llidar|[f]oxglove|[r]osbag2" || echo "  (乾淨)"
  BUSY=$(sudo fuser /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null | tr -d " ")
  [ -n "$BUSY" ] && echo "⚠ 序列埠仍被佔用：$BUSY" || echo "序列埠：沒人佔用"
  exit 0
fi

# ---------------- start ----------------
if [ "$CMD" = "start" ]; then
  LOAD_PG=""
  START_POSE="0.0,0.0,0.0"
  SLAM_NODE="async"          # 續建也預設 async，理由見下方起 slam_toolbox 處的註解
  while [ $# -gt 0 ]; do
    case "$1" in
      --load) LOAD_PG="$2"; shift 2 ;;
      --start-pose) START_POSE="$2"; shift 2 ;;
      --slam-node) SLAM_NODE="$2"; shift 2 ;;
      *) echo "未知參數：$1"; exit 2 ;;
    esac
  done
  case "$SLAM_NODE" in async|lifelong) ;; *) echo "--slam-node 只接受 async|lifelong"; exit 2 ;; esac

  if [ -n "$LOAD_PG" ] && [ ! -f "${LOAD_PG}.posegraph" ]; then
    echo "❌ 找不到要載入的姿態圖：${LOAD_PG}.posegraph"; exit 2
  fi

  # 開跑前一定要乾淨：殘留節點會兩份同時發 /map、/scan、搶同一個序列埠，量到的全是垃圾。
  # ⚠ 殺 launch 母行程「不會」帶走它的子節點——子節點被 init 收養繼續跑（repo 資料地雷）。
  #   所以要逐個節點執行檔點名。⚠ 執行檔叫 `bridge` 不是 `bridge_node`（2026-08-30 踩到：
  #   模式寫錯 → 兩個 bridge 同時開 /dev/stm32 → 資料被撕成兩半、/scan 與 /map 全空）。
  cleanup_nodes
  sleep 3

  # 清完必須「驗證」真的乾淨，不能假設 pkill 成功——這道檢查就是上面那次事故缺的。
  BUSY=$(sudo fuser /dev/ttyUSB0 /dev/ttyUSB1 2>/dev/null | tr -d " ")
  if [ -n "$BUSY" ]; then
    echo "❌ 序列埠仍被佔用（PID: $BUSY），拒絕開跑——硬起下去會兩套節點搶埠"
    sudo fuser -v /dev/ttyUSB0 /dev/ttyUSB1 2>&1 | tail -5
    exit 3
  fi

  # ⚠ 目的地已存在＝拒跑，不覆蓋也不沿用（比照 map_from_bag.sh 的設計約束 b）。
  #   2026-08-30 踩到：對帳失敗後重跑 start，ros2 bag record 因「資料夾已存在」啟動失敗，
  #   腳本卻照樣說「可以開車了」→ 整趟沒錄到 bag，事後才發現保險是空的。
  if [ -e "$RUNDIR" ]; then
    echo "❌ $RUNDIR 已存在。換個名稱，或先確認裡面的東西不要了再手動刪除。"
    echo "   （直接沿用會讓 bag 錄製靜默失敗——那是這趟唯一的重跑保險）"
    exit 5
  fi
  mkdir -p "$RUNDIR"

  echo "▶ 起 bringup（光達＋橋接＋EKF）"
  setsid nohup ros2 launch stm32_bridge bringup.launch.py lidar_type:=c1 \
    > "$RUNDIR/bringup.log" 2>&1 </dev/null & disown
  sleep 15

  if [ -n "$LOAD_PG" ]; then
    IFS=, read -r SPX SPY SPYAW <<< "$START_POSE"
    # ⚠ 一律格式化成小數：ROS 會把 [0, 0, 0] 解析成整數陣列，而 map_start_pose 要的是
    #   浮點陣列 → configure 直接失敗（2026-08-30 踩到，錯誤訊息埋在 slam.log 裡不易發現）。
    SPX=$(printf '%.6f' "$SPX"); SPY=$(printf '%.6f' "$SPY"); SPYAW=$(printf '%.6f' "$SPYAW")
    echo "▶ 起 slam_toolbox（$SLAM_NODE，載入 $LOAD_PG，起始位姿 $SPX,$SPY,$SPYAW）"
    # ⚠ 續建節點怎麼選（2026-08-30 RM2 實車推翻 RM1 的結論，預設改為 async）：
    #   RM1（小圖，8→18 節點）：lifelong 較好；async 會把牆疊成雙線（94.7%→51%）。
    #   RM2（大圖，配到 116 個節點編號）：**lifelong 會把姿態圖裁到碎裂**——
    #     第一遍 17 個節點被裁到剩 5 個，刪節點時連邊一起刪、不重接鄰居，
    #     整張圖碎成 24 個互不相連的塊，**跨遍約束歸零**（驗收條件①直接失敗）。
    #   同一顆 bag 離線對照（只換節點類型，其餘完全相同）：
    #     async    64 節點 101 邊／跨遍約束 11 條／第一遍牆保留 98.1%／一致率 99.5%
    #     lifelong 54 節點  33 邊／跨遍約束  0 條／第一遍牆保留 97.4%／一致率 99.2%
    #   且 RM1 擔心的雙線現象在 RM2 沒有出現（牆連貫度 29.6%，第一遍自己是 27.7%，沒掉）。
    #   ⇒ 大圖用 async；要復現 RM1 的行為請明確帶 --slam-node lifelong。
    setsid nohup ros2 run slam_toolbox "${SLAM_NODE}_slam_toolbox_node" --ros-args \
      -p odom_frame:=odom -p base_frame:=base_link -p map_frame:=map \
      -p scan_topic:=/scan -p mode:=mapping -p use_sim_time:=false \
      -p map_file_name:="$LOAD_PG" -p "map_start_pose:=[$SPX, $SPY, $SPYAW]" \
      -p map_start_at_dock:=false \
      > "$RUNDIR/slam.log" 2>&1 </dev/null & disown
  else
    echo "▶ 起 slam_toolbox（async，從零建圖）"
    setsid nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
      -p odom_frame:=odom -p base_frame:=base_link -p map_frame:=map \
      -p scan_topic:=/scan -p mode:=mapping -p use_sim_time:=false \
      > "$RUNDIR/slam.log" 2>&1 </dev/null & disown
  fi
  sleep 8

  ros2 lifecycle set /slam_toolbox configure > "$RUNDIR/lifecycle.log" 2>&1 || {
    echo "❌ slam_toolbox configure 失敗，見 $RUNDIR/slam.log"; tail -5 "$RUNDIR/slam.log"; exit 6; }
  sleep 2
  ros2 lifecycle set /slam_toolbox activate >> "$RUNDIR/lifecycle.log" 2>&1 || {
    echo "❌ slam_toolbox activate 失敗，見 $RUNDIR/slam.log"; tail -5 "$RUNDIR/slam.log"; exit 6; }
  sleep 3

  echo "▶ 起 foxglove_bridge（port 8765，供筆電看圖＋遙控）"
  setsid nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 \
    > "$RUNDIR/foxglove.log" 2>&1 </dev/null & disown

  echo "▶ 起 bag 錄製（保險：live 跑壞了還能離線重建，不必再開一次車）"
  setsid nohup ros2 bag record -s mcap -o "$RUNDIR/bag" \
    /odom /imu/data /odometry/filtered /scan /tf /tf_static /cmd_vel \
    > "$RUNDIR/rec.log" 2>&1 </dev/null & disown
  sleep 5
  # 錄製是唯一的重跑保險，啟動失敗必須當場擋下，不能等事後才發現
  if ! [ -d "$RUNDIR/bag" ] || ! ls "$RUNDIR"/bag/*.mcap >/dev/null 2>&1; then
    echo "❌ bag 錄製沒起來（見 $RUNDIR/rec.log）"; tail -3 "$RUNDIR/rec.log"; exit 5
  fi

  echo ""
  echo "===== 開跑前對帳 ====="
  echo "節點："; ros2 node list 2>/dev/null | sed 's/^/  /'
  # 發布者數必須恰好 1——多於 1 ＝ 殘留節點沒清乾淨，資料會互相污染（本卡踩過）
  RC=0
  for t in /scan /odom /odometry/filtered; do
    N=$(timeout 15 ros2 topic info "$t" 2>/dev/null | awk "/Publisher count/{print \$3}")
    printf "%-20s 發布者 %s " "$t" "${N:-?}"
    if [ "${N:-0}" = "1" ]; then echo "✅"; else echo "❌（應為 1）"; RC=1; fi
  done
  # ⚠ 不可寫成「管線 || 失敗」：set -o pipefail 下 timeout 的 124 會蓋掉 grep 的成功，
  #   變成「明明有資料卻報沒有」的假失敗（2026-08-30 踩到）。改成抓輸出再判空。
  HZ=$(timeout 12 ros2 topic hz /scan 2>&1 | grep -m1 "average rate" || true)
  printf "%-20s %s " "/scan 頻率" "${HZ:-}"
  if [ -n "$HZ" ]; then echo "✅"; else echo "❌ 沒有資料"; RC=1; fi
  W=$(timeout 15 ros2 topic echo /map --once --field info.width 2>/dev/null | head -1 || true)
  printf "%-20s %s " "/map 寬度" "${W:-}"
  if [ -n "$W" ]; then echo "✅"; else echo "❌ 沒有資料"; RC=1; fi
  if [ "$RC" != "0" ]; then
    echo ""
    echo "❌ 開跑前對帳失敗——先修好再開車，不要帶著壞狀態錄一趟"
    exit 4
  fi
  echo "產物目錄：$RUNDIR"
  echo "✅ 可以開車了"
  exit 0
fi

# ---------------- finish ----------------
if [ "$CMD" = "finish" ]; then
  [ -d "$RUNDIR" ] || { echo "❌ 找不到 $RUNDIR，這趟沒 start 過？"; exit 2; }

  echo "▶ 停 bag（用 TERM，SIGINT 對 detached recorder 常無效、不會 finalize metadata）"
  pkill -TERM -f "bag record" >/dev/null 2>&1
  sleep 4

  echo "▶ 存姿態圖（.posegraph + .data）"
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$RUNDIR/$NAME'}" > "$RUNDIR/serialize.log" 2>&1
  for _ in $(seq 1 30); do [ -f "$RUNDIR/$NAME.posegraph" ] && break; sleep 1; done
  [ -f "$RUNDIR/$NAME.posegraph" ] || { echo "❌ 沒產出 $NAME.posegraph，見 $RUNDIR/serialize.log"; exit 8; }
  [ -f "$RUNDIR/$NAME.data" ]      || { echo "❌ 缺 $NAME.data"; exit 8; }

  echo "▶ 存地圖（.pgm + .yaml）"
  # ⚠ 預設等待只有 2s 會失敗，必須加 save_map_timeout（2026-07-20 踩過）
  ros2 run nav2_map_server map_saver_cli -f "$RUNDIR/$NAME" \
    --ros-args -p save_map_timeout:=30.0 > "$RUNDIR/save.log" 2>&1
  [ -f "$RUNDIR/$NAME.pgm" ] || { echo "❌ 沒產出 $NAME.pgm，見 $RUNDIR/save.log"; exit 9; }

  echo "▶ 抓姿態圖快照（跨遍約束的證據來源）"
  python3 "$TOOL" snapshot --out "$RUNDIR/graph.json" 2>&1 | sed 's/^/  /'

  echo "▶ 收乾淨"
  cleanup_nodes
  sleep 3

  echo ""
  echo "===== 這趟的產物 ====="
  ls -la "$RUNDIR" | grep -vE "^total|^d.*\.$" | awk '{printf "  %8s  %s\n", $5, $9}'
  echo "殘留行程（應為 0）：$(ps -eo args | grep -cE "[s]lam_toolbox_node|[b]ringup.launch")"
  exit 0
fi

echo "未知指令：$CMD"; exit 2
