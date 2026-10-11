#!/bin/bash
# 從 bag 離線重建地圖；支援「載入既有姿態圖續建」（RM 系列二遍續建）。
#
# 用法：
#   map_from_bag.sh --bag <路徑> --lidar <c1|rplidar|ydlidar> --out <名稱> [選項]
# 選項：
#   --offset <秒>            回放起始偏移（預設 0）
#   --duration <秒>          回放時長（預設 -1＝整顆放完）
#   --serialize              建圖結束後存 .posegraph（第一遍要加）
#   --load-posegraph <前綴>  載入既有姿態圖續建（第二遍用；前綴＝不含 .posegraph）
#   --start-pose <x,y,yaw>   續建時車在舊圖座標系的起始位姿（弧度）。搭配 --load-posegraph；
#                            省略＝(0,0,0)。⚠ 給錯會讓第二遍的掃描匹配把第一遍的好圖塗壞。
#   --skip-hash              跳過來源 bag 雜湊（趕時間用；產物 manifest 會標記未驗證）
#   --slam-node <async|lifelong>
#                            建圖節點（預設 async）。lifelong＝slam_toolbox 專為「反覆重建
#                            同一區域」設計的節點，會裁剪舊節點並重新優化；async 續建時
#                            實測會把兩趟的牆疊成雙線（2026-08-10 RM1）。
#
# 設計約束（2026-08-08 packet RM1 審查要求，逐項對應）：
#   (a) 光達安裝值**必填且來自單一真相源** config/lidar_profiles.json，未知型號＝拒跑。
#       舊版寫死退役 X4 的 0.123/0.185/yaw=0，拿去重建 C1 的 bag 會讓整張圖歪掉且不報錯。
#   (b) 每次跑用**獨立 run 目錄**（帶 run id），目的地已存在＝拒跑不覆蓋。舊版 rm -rf 直接蓋掉。
#   (c) 只收自己啟動的行程（**process group 級**，非全域 pgrep）；偵測到不屬於本 run 的
#       slam/bag/ekf 行程＝拒跑，不代為終止（可能是別人正在錄實車）。
#   (d) **失敗一律非零退出**；只有全部檢核通過才印 DONE。舊版最後一行無條件 echo DONE。
# ⚠ set -u 必須在 source ROS setup 之後：setup.bash 會讀 AMENT_TRACE_SETUP_FILES 等
# 未定義變數，提前開 -u 會讓整支腳本在 source 那行就爆掉（舊版註解已記，2026-08-10 重踩一次）。
set -o pipefail

die() { echo "FATAL: $*" >&2; exit "${2:-1}"; }

# ---------- 參數 ----------
BAG=""; LIDAR=""; OUT=""; OFF=0; DUR=-1; SERIALIZE=0; LOAD_PG=""; START_POSE="0.0,0.0,0.0"; SKIP_HASH=0
SLAM_NODE="async"
while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAG="${2:-}"; shift 2 ;;
    --lidar) LIDAR="${2:-}"; shift 2 ;;
    --out) OUT="${2:-}"; shift 2 ;;
    --offset) OFF="${2:-}"; shift 2 ;;
    --duration) DUR="${2:-}"; shift 2 ;;
    --serialize) SERIALIZE=1; shift ;;
    --load-posegraph) LOAD_PG="${2:-}"; shift 2 ;;
    --start-pose) START_POSE="${2:-}"; shift 2 ;;
    --skip-hash) SKIP_HASH=1; shift ;;
    --slam-node) SLAM_NODE="${2:-}"; shift 2 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) die "未知參數：$1（用 --help 看用法）" 2 ;;
  esac
done
[ -n "$BAG" ]   || die "缺 --bag" 2
[ -n "$LIDAR" ] || die "缺 --lidar（c1/rplidar/ydlidar）——本腳本不提供預設值，見設計約束 (a)" 2
[ -n "$OUT" ]   || die "缺 --out" 2
case "$SLAM_NODE" in async|lifelong) ;; *) die "--slam-node 只接受 async|lifelong：$SLAM_NODE" 2 ;; esac
[ -d "$BAG" ]   || die "bag 目錄不存在：$BAG" 2
[ -f "$BAG/metadata.yaml" ] || die "不像 rosbag2 目錄（缺 metadata.yaml）：$BAG" 2

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
set -u   # 見檔頭：必須在 source 之後才能開

PROFILES=~/ros2_ws/src/stm32_bridge/config/lidar_profiles.json
[ -f "$PROFILES" ] || die "找不到光達 profile 檔：$PROFILES" 2

# ---------- (a) 光達 profile：未知型號 fail-closed ----------
read -r LX LZ LYAW < <(python3 - "$PROFILES" "$LIDAR" <<'PY'
import json, sys
prof = json.load(open(sys.argv[1]))["profiles"]
key = sys.argv[2]
if key not in prof:
    sys.stderr.write("未知光達型號 %r，可選：%s\n" % (key, ", ".join(sorted(prof))))
    sys.exit(3)
p = prof[key]
print(p["laser_x"], p["laser_z"], p["laser_yaw"])
PY
) || die "光達 profile 解析失敗（型號 $LIDAR）" 3
echo "光達 profile: $LIDAR  x=$LX z=$LZ yaw=$LYAW"

# ---------- 續建參數檢查 ----------
if [ -n "$LOAD_PG" ]; then
  [ -f "${LOAD_PG}.posegraph" ] || die "找不到姿態圖：${LOAD_PG}.posegraph" 2
  [ -f "${LOAD_PG}.data" ]      || die "找不到姿態圖資料：${LOAD_PG}.data" 2
  echo "$START_POSE" | grep -Eq '^-?[0-9.]+,-?[0-9.]+,-?[0-9.]+$' \
    || die "--start-pose 格式須為 x,y,yaw（弧度）：$START_POSE" 2
fi

# ---------- (b) 獨立 run 目錄，不覆蓋 ----------
RUNID="$(date +%Y%m%d_%H%M%S)_$$"
RUNDIR=~/cmpre/"${OUT}_${RUNID}"
[ -e "$RUNDIR" ] && die "run 目錄已存在，拒絕覆蓋：$RUNDIR" 2
mkdir -p "$RUNDIR" || die "無法建立 run 目錄：$RUNDIR"
echo "run 目錄: $RUNDIR"

# ---------- (c) 衝突偵測：不屬於本 run 的相關行程一律拒跑 ----------
CONFLICT=""
for pat in "[s]lam_toolbox" "[b]ag play" "[m]ap_saver" "[e]kf_node" "[b]ag record"; do
  hits="$(pgrep -f "$pat" || true)"
  [ -n "$hits" ] && CONFLICT="$CONFLICT $pat($hits)"
done
if [ -n "$CONFLICT" ]; then
  die "偵測到不屬於本 run 的行程，拒跑（可能有人正在錄實車；請自行確認後手動處理）：$CONFLICT" 4
fi

# ---------- process group 管理：只收自己啟動的 ----------
PGIDS=()
spawn() {  # spawn <log名> <指令...>
  local log="$RUNDIR/$1"; shift
  local pidfile; pidfile="$(mktemp)"
  setsid bash -c 'echo $$ > "$1"; shift; exec "$@"' _ "$pidfile" "$@" \
      > "$log" 2>&1 < /dev/null &
  for _ in $(seq 1 50); do [ -s "$pidfile" ] && break; sleep 0.1; done
  local pgid; pgid="$(cat "$pidfile" 2>/dev/null || true)"; rm -f "$pidfile"
  [ -n "$pgid" ] || die "子行程啟動失敗（取不到 PGID）：$1" 5
  PGIDS+=("$pgid")
  echo "$pgid"
}

cleanup() {
  local rc=$?                       # 保留原退出碼（(d)：清理不得吃掉失敗）
  for pgid in "${PGIDS[@]:-}"; do
    [ -n "$pgid" ] || continue
    kill -TERM -"$pgid" 2>/dev/null || true
  done
  for _ in $(seq 1 20); do
    local alive=0
    for pgid in "${PGIDS[@]:-}"; do
      [ -n "$pgid" ] && kill -0 -"$pgid" 2>/dev/null && alive=1
    done
    [ "$alive" = 0 ] && break
    sleep 0.5
  done
  for pgid in "${PGIDS[@]:-}"; do
    [ -n "$pgid" ] && kill -KILL -"$pgid" 2>/dev/null || true
  done
  exit "$rc"
}
trap cleanup EXIT INT TERM

# ---------- 起 TF / EKF ----------
spawn statictf.log ros2 run tf2_ros static_transform_publisher \
  --x "$LX" --y 0 --z "$LZ" --yaw "$LYAW" --pitch 0 --roll 0 \
  --frame-id base_link --child-frame-id laser_frame \
  --ros-args -p use_sim_time:=true > /dev/null

spawn ekf.log ros2 run robot_localization ekf_node --ros-args \
  --params-file ~/ros2_ws/src/stm32_bridge/config/ekf.yaml -p use_sim_time:=true > /dev/null
sleep 3

spawn rec.log ros2 bag record -s mcap -o "$RUNDIR/odom" /odometry/filtered > /dev/null
sleep 2

# ---------- slam_toolbox（續建時載入舊姿態圖） ----------
SLAM_ARGS=(-p use_sim_time:=true -p odom_frame:=odom -p base_frame:=base_link
           -p map_frame:=map -p scan_topic:=/scan -p mode:=mapping)
if [ -n "$LOAD_PG" ]; then
  IFS=, read -r SPX SPY SPYAW <<< "$START_POSE"
  # ⚠ 一律格式化成小數：ROS 會把 [0, 0, 0] 解析成整數陣列，而 map_start_pose 要的是
  #   浮點陣列 → configure 直接失敗（2026-08-30 踩到，錯誤訊息埋在 slam.log 裡不易發現）。
  SPX=$(printf '%.6f' "$SPX"); SPY=$(printf '%.6f' "$SPY"); SPYAW=$(printf '%.6f' "$SPYAW")
  SLAM_ARGS+=(-p map_file_name:="$LOAD_PG"
              -p "map_start_pose:=[$SPX, $SPY, $SPYAW]"
              -p map_start_at_dock:=false)
  echo "續建模式：載入 $LOAD_PG，起始位姿 ($SPX, $SPY, $SPYAW)"
fi
spawn slam.log ros2 run slam_toolbox "${SLAM_NODE}_slam_toolbox_node" --ros-args "${SLAM_ARGS[@]}" > /dev/null
sleep 6

ros2 lifecycle set /slam_toolbox configure > "$RUNDIR/lifecycle.log" 2>&1 \
  || die "slam_toolbox configure 失敗（見 $RUNDIR/slam.log）" 6
sleep 2
ros2 lifecycle set /slam_toolbox activate >> "$RUNDIR/lifecycle.log" 2>&1 \
  || die "slam_toolbox activate 失敗（見 $RUNDIR/slam.log）" 6

# ---------- 回放：只放感測原料，不放舊 /tf（舊 EKF yaw 死區 bug）----------
PLAY=(ros2 bag play "$BAG" --clock 100 --rate 1.0 --topics /odom /imu/data /scan --start-offset "$OFF")
[ "$DUR" != "-1" ] && PLAY+=(--playback-duration "$DUR")
"${PLAY[@]}" > "$RUNDIR/play.log" 2>&1 || die "bag play 失敗（見 $RUNDIR/play.log）" 7
sleep 3

# ---------- 存姿態圖（第一遍）----------
if [ "$SERIALIZE" = 1 ]; then
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$RUNDIR/$OUT'}" > "$RUNDIR/serialize.log" 2>&1 \
    || die "serialize_map 呼叫失敗（見 $RUNDIR/serialize.log）" 8
  for _ in $(seq 1 30); do [ -f "$RUNDIR/$OUT.posegraph" ] && break; sleep 1; done
  [ -f "$RUNDIR/$OUT.posegraph" ] || die "serialize 呼叫回傳成功但沒產出 $OUT.posegraph" 8
  [ -f "$RUNDIR/$OUT.data" ]      || die "serialize 缺 $OUT.data" 8
fi

# ---------- 存圖 ----------
ros2 run nav2_map_server map_saver_cli -f "$RUNDIR/$OUT" \
  --ros-args -p save_map_timeout:=30.0 > "$RUNDIR/save.log" 2>&1 \
  || die "map_saver 失敗（見 $RUNDIR/save.log）" 9
[ -f "$RUNDIR/$OUT.pgm" ] && [ -f "$RUNDIR/$OUT.yaml" ] || die "map_saver 回傳成功但沒有圖檔" 9

# 錄影器要 TERM 才會寫 metadata（runbook 已知坑）——這裡先單獨收，確認落盤
kill -TERM -"${PGIDS[2]}" 2>/dev/null || true
for _ in $(seq 1 20); do [ -f "$RUNDIR/odom/metadata.yaml" ] && break; sleep 1; done
[ -f "$RUNDIR/odom/metadata.yaml" ] || die "odom 錄製沒有 metadata.yaml（未正常 finalize）" 10

# ---------- manifest（證據鏈：來源、參數、產物）----------
if [ "$SKIP_HASH" = 1 ]; then BAGHASH="skipped"; else
  BAGHASH="$(find "$BAG" -type f -exec sha256sum {} + | sort -k2 | sha256sum | cut -d' ' -f1)" \
    || die "來源 bag 雜湊失敗" 11
fi
python3 - "$RUNDIR/manifest.json" <<PY || die "manifest 寫入失敗" 11
import json, sys
json.dump({
    "run_id": "$RUNID", "out": "$OUT", "run_dir": "$RUNDIR",
    "source_bag": "$BAG", "source_bag_sha256": "$BAGHASH",
    "lidar_profile": "$LIDAR", "laser_x": $LX, "laser_z": $LZ, "laser_yaw": $LYAW,
    "slam_node": "$SLAM_NODE",
    "offset_s": $OFF, "duration_s": $DUR,
    "serialized": bool($SERIALIZE),
    "loaded_posegraph": "$LOAD_PG" or None, "start_pose": "$START_POSE" if "$LOAD_PG" else None,
}, open(sys.argv[1], "w"), ensure_ascii=False, indent=2)
PY

echo "map_saver_rc=0  odom_rec=OK"
ls -la "$RUNDIR/$OUT.pgm" "$RUNDIR/$OUT.yaml"
[ "$SERIALIZE" = 1 ] && ls -la "$RUNDIR/$OUT.posegraph" "$RUNDIR/$OUT.data"
echo "DONE_${OUT} run=${RUNID}"
