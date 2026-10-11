#!/bin/bash
# EX3 故障注入第 4 項：強殺 supervisor 本體，量測子節點被 kernel 收殺的延遲。
# 寫成腳本執行，避免 pgrep -f 比對到呼叫端自己的指令列。
# ⚠ 不用 set -u：ROS 的 setup.bash 會讀未設定的變數（AMENT_TRACE_SETUP_FILES）而直接中止
LOG=/tmp/ex3_orphan_result.txt
: > "$LOG"

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 起 bringup（帶 pdeathsig 前綴），車不會動（沒有 Nav2）
setsid nohup ros2 launch stm32_bridge bringup.launch.py \
    node_prefix:="setpriv --pdeathsig=SIGKILL" >/tmp/ex3_orphan_bringup.log 2>&1 &
sleep 20

LAUNCH_PID=$(pgrep -P $$ -f 'ros2' | head -1)
[ -z "$LAUNCH_PID" ] && LAUNCH_PID=$(ps -eo pid,cmd --no-headers |
    awk '/ros2 launch stm32_bridge bringup/ && !/awk/ {print $1; exit}')

CHILDREN=$(pgrep -P "$LAUNCH_PID" | tr '\n' ' ')
{
  echo "launch PID = $LAUNCH_PID"
  echo "子節點 PID = $CHILDREN"
  for p in $CHILDREN; do
      printf '  %s %s\n' "$p" "$(cat /proc/$p/comm 2>/dev/null)"
  done
  echo
} >> "$LOG"

if [ -z "$CHILDREN" ]; then
  echo "FAIL：抓不到子節點，測試無效" >> "$LOG"
  cat "$LOG"; exit 1
fi

kill -9 "$LAUNCH_PID"
T0=$(date +%s.%N)

echo "強殺 launch 之後，各子節點消失的時間：" >> "$LOG"
REMAIN="$CHILDREN"
for _ in $(seq 1 120); do          # 最多等 30 秒
    NEW=""
    for p in $REMAIN; do
        if kill -0 "$p" 2>/dev/null; then
            NEW="$NEW $p"
        else
            NOW=$(date +%s.%N)
            printf '  PID %-6s %s  於 %.2f 秒後消失\n' \
                "$p" "$(echo $p | xargs -I{} echo {})" \
                "$(echo "$NOW - $T0" | bc)" >> "$LOG"
        fi
    done
    REMAIN=$(echo $NEW | xargs)
    [ -z "$REMAIN" ] && break
    sleep 0.25
done

echo >> "$LOG"
if [ -z "$REMAIN" ]; then
    echo "RESULT=PASS 全部子節點都被收殺，零孤兒" >> "$LOG"
else
    echo "RESULT=FAIL 30 秒後仍有孤兒：$REMAIN" >> "$LOG"
    for p in $REMAIN; do
        echo "  $p 狀態=$(awk '{print $3}' /proc/$p/stat 2>/dev/null)" >> "$LOG"
        kill -9 "$p" 2>/dev/null
    done
fi
cat "$LOG"
