#!/bin/bash
# EX3 故障注入：讓輪子轉起來，然後用指定方式破壞系統，量測各節點死亡時間。
# 用法：bash ex3_inject.sh <sigint|kill-explorer|kill-controller|kill-supervisor>
#
# ⚠ 不用 set -u（ROS setup.bash 會讀未設定變數）。
# ⚠ 指令列不含被 pgrep 比對的關鍵字，避免自我匹配（專案 runbook 的坑）。
MODE="${1:-sigint}"
# 第二參數＝本次 supervisor 的 log 路徑。
# ⚠ 不可用 /tmp/ex3_run*.log 萬用字元抓——會匹配到歷史 log，讀到上一次的結果。
# 2026-08-11 踩過：連續兩項注入回報「完全相同的 0.214 秒」，分毫不差本身就是證據。
RUNLOG="${2:-/tmp/ex3_run.log}"
LOG=/tmp/ex3_inject_${MODE}.txt
: > "$LOG"

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# ⚠ pattern 一律用 ^ 錨定到執行檔的完整路徑。
# 只比對片語（如 'ros2 launch ... explore'）會匹配到「呼叫這個腳本的那條 ssh 指令列」，
# 於是抓到自己的 shell、算出自己的行程群組，再把 SIGINT 送給自己＝腳本自殺。
# 2026-08-11 實測踩過：抓到 PID 4101（實際是 4109），PGID 737 是 ssh session 的群組。
pid_of() { pgrep -f "$1" | head -1; }

LAUNCH_PID=$(pid_of '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch stm32_bridge explore')
BRIDGE_PID=$(pid_of '^/usr/bin/python3 .*/stm32_bridge/bridge')
EXPLORER_PID=$(pid_of '^/usr/bin/python3 .*/frontier_explorer')
CTRL_PID=$(pid_of '^/opt/ros/jazzy/lib/nav2_controller/controller_server')

# 防呆：不論如何都不能對自己所屬的行程群組動手
MY_PGID=$(ps -o pgid= -p $$ | tr -d ' ')

{
  echo "=== 注入模式：$MODE ==="
  echo "supervisor(launch) PID = $LAUNCH_PID"
  echo "bridge PID            = $BRIDGE_PID   ← 唯一能讓車動的節點"
  echo "explorer PID          = $EXPLORER_PID"
  echo "controller_server PID = $CTRL_PID"
  echo
} >> "$LOG"

if [ -z "$LAUNCH_PID" ] || [ -z "$BRIDGE_PID" ]; then
  echo "FAIL：系統沒起來，測試無效" >> "$LOG"; cat "$LOG"; exit 1
fi

# --- 讓輪子轉起來：持續灌 0.15 m/s 前進指令 ---
# 這個發送端刻意不歸 supervisor 管，注入後它還會繼續叫車前進，
# 用來證明「就算有人一直催，車照樣停」。
setsid nohup ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.15}, angular: {z: 0.0}}' >/tmp/ex3_cmdpub.log 2>&1 &
CMD_PID=$!
sleep 4

echo "--- 注入前的輪速（即時讀 /odom，應為非零＝輪子在轉）---" >> "$LOG"
# ⚠ 不要 grep launch 的 log 檔：stdout 重導到檔案有緩衝，會讀到好幾秒前的舊值
SPEED=$(python3 $HOME/ros2_ws/tools/read_speed.py 3)
echo "  $SPEED" >> "$LOG"
case "$SPEED" in
  *SPEED_MAX=none*|*SPEED_MAX=0.00*)
      echo "  ⚠ 輪速為零——車沒在動，這次注入測不到東西" >> "$LOG" ;;
esac
echo >> "$LOG"

# --- 注入故障 ---
T0=$(date +%s.%N)
case "$MODE" in
  sigint)
      # 真正的 Ctrl-C 是把 SIGINT 送給前景「行程群組」。只送給 launch 這一個 PID
      # 完全沒作用（2026-08-11 實測，40 秒後所有節點照樣活著）。
      PGID=$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')
      if [ "$PGID" = "$MY_PGID" ] || [ -z "$PGID" ]; then
          echo "  FAIL：算出的行程群組($PGID)等於自己的($MY_PGID)，拒絕執行" >> "$LOG"
          kill -9 "$CMD_PID" 2>/dev/null; cat "$LOG"; exit 1
      fi
      echo "  （送 SIGINT 給行程群組 -$PGID，等同終端按 Ctrl-C）" >> "$LOG"
      kill -INT -"$PGID" ;;
  kill-explorer)   [ -n "$EXPLORER_PID" ] && kill -9 "$EXPLORER_PID" ;;
  kill-controller) [ -n "$CTRL_PID" ] && kill -9 "$CTRL_PID" ;;
  kill-supervisor) kill -9 "$LAUNCH_PID" ;;
  *) echo "未知模式" >> "$LOG"; cat "$LOG"; exit 1 ;;
esac
echo "已於 T0 注入：$MODE" >> "$LOG"

# --- 量測各節點死亡時間（bridge 死＝serial 斷＝韌體 watchdog 接手）---
WATCH="$BRIDGE_PID $EXPLORER_PID $CTRL_PID $LAUNCH_PID"
REMAIN=$(echo $WATCH | xargs -n1 | sort -u | xargs)
echo "--- 各行程消失時間 ---" >> "$LOG"
for _ in $(seq 1 160); do        # 最多 40 秒
    NEW=""
    for p in $REMAIN; do
        if kill -0 "$p" 2>/dev/null; then NEW="$NEW $p"; else
            NOW=$(date +%s.%N)
            NAME=bridge
            [ "$p" = "$EXPLORER_PID" ] && NAME=explorer
            [ "$p" = "$CTRL_PID" ] && NAME=controller_server
            [ "$p" = "$LAUNCH_PID" ] && NAME=supervisor
            printf '  %-18s PID %-6s 於 %.2f 秒後消失\n' "$NAME" "$p" \
                "$(echo "$NOW - $T0" | bc)" >> "$LOG"
        fi
    done
    REMAIN=$(echo $NEW | xargs)
    [ -z "$REMAIN" ] && break
    sleep 0.25
done
[ -n "$REMAIN" ] && echo "  ⚠ 40 秒後仍存活：$REMAIN" >> "$LOG"

# --- 收掉指令發送端 ---
kill -9 "$CMD_PID" 2>/dev/null
pkill -9 -f 'topic pub -r 10' 2>/dev/null
sleep 1

echo >> "$LOG"
# --- 卡上真正的驗收條件：FBK 輪速歸零 <=1 秒（Codex F3，2026-08-11）---
# bridge 關閉前會持續讀 FBK 直到輪速歸零，並印 STOP_CONFIRMED: ... 於 X 秒。
# 這是唯一能以同一時基量到「車真的停了」的位置——bridge 一死就沒人在讀 FBK。
echo "--- 停車確認（驗收條件：<=1.0 秒）---" >> "$LOG"
if [ "$MODE" = "kill-supervisor" ]; then
    # ⚠ 這一項**物理上不可能有軟體量測**：pdeathsig 用 SIGKILL 殺 bridge，
    #   SIGKILL 不可捕捉，bridge 沒有任何機會執行停車確認。此時車靠的是
    #   STM32 韌體的 500 ms watchdog（serial 一斷就停），那發生在 MCU 裡，
    #   Pi 上拿不到證據。故本項以「bridge 消失時刻 + 韌體 watchdog 規格」推導，
    #   **標示為 DERIVED，不可與其他項的 MEASURED 混為一談**。
    # 用 awk 抽欄位，不要用 grep 硬比對空白排版（欄位是用 printf 對齊的，空格數不固定）
    BR=$(awk '/bridge/ && /消失/ {for(i=1;i<=NF;i++) if($i=="於") {print $(i+1); exit}}' "$LOG")
    if [ -n "$BR" ]; then
        TOTAL=$(echo "$BR + 0.5" | bc)
        echo "  bridge 於 ${BR}s 消失（serial 斷）＋韌體 watchdog 0.5s（規格，v74 已驗證）" >> "$LOG"
        if [ "$(echo "$TOTAL <= 1.0" | bc)" = "1" ]; then
            echo "  VERDICT=PASS(DERIVED) 推導車停時刻 ${TOTAL}s <= 1.0s" >> "$LOG"
        else
            echo "  VERDICT=FAIL(DERIVED) 推導車停時刻 ${TOTAL}s > 1.0s" >> "$LOG"
        fi
    else
        echo "  VERDICT=FAIL 抓不到 bridge 消失時刻" >> "$LOG"
    fi
    echo >> "$LOG"
    echo "--- 殘留檢查（控制類行程應全滅）---" >> "$LOG"
    LEFT=$(ps -eo pid,cmd --no-headers | grep -E 'stm32_bridge/(bridge|frontier_explorer)|nav2_(controller|planner|bt_navigator)|slam_toolbox' | grep -v grep | wc -l)
    echo "  殘留控制類行程數 = $LEFT" >> "$LOG"
    cat "$LOG"
    exit 0
fi
STOPLINE=$(grep -h "STOP_CONFIRMED\|STOP_UNCONFIRMED" "$RUNLOG" 2>/dev/null | tail -1)
if [ -z "$STOPLINE" ]; then
    echo "  VERDICT=FAIL 找不到 STOP_CONFIRMED/STOP_UNCONFIRMED（bridge 版本太舊？）" >> "$LOG"
elif echo "$STOPLINE" | grep -q STOP_UNCONFIRMED; then
    echo "  VERDICT=FAIL bridge 未能在時限內確認輪速歸零：" >> "$LOG"
    echo "    $STOPLINE" >> "$LOG"
else
    SEC=$(echo "$STOPLINE" | grep -oE 'stop_time_s=[0-9.]+' | cut -d= -f2)
    echo "    $STOPLINE" >> "$LOG"
    if [ -n "$SEC" ] && [ "$(echo "$SEC <= 1.0" | bc)" = "1" ]; then
        echo "  VERDICT=PASS 輪速歸零 ${SEC}s <= 1.0s" >> "$LOG"
    else
        echo "  VERDICT=FAIL 輪速歸零 ${SEC}s > 1.0s" >> "$LOG"
    fi
fi

echo >> "$LOG"
echo "--- 殘留檢查（控制類行程應全滅）---" >> "$LOG"
LEFT=$(ps -eo pid,cmd --no-headers | grep -E 'stm32_bridge/(bridge|frontier_explorer)|nav2_(controller|planner|bt_navigator)|slam_toolbox' | grep -v grep | wc -l)
echo "  殘留控制類行程數 = $LEFT" >> "$LOG"
cat "$LOG"
