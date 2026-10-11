#!/usr/bin/env python3
"""EX4／EX3 限速驗收：比對「Nav2 下的指令速度」與「實際量到的輪速」。

**為什麼一定要兩個都看**：FBK 回報的是**量到的**輪速，Nav2 發的 `/cmd_vel` 是**指令**。
兩者超標的意義完全不同、修法也不同：

  * 指令就超標 ⇒ Nav2 沒有硬性上限（`desired_linear_vel` 只是「希望的巡航速度」，
    不是 cap；轉彎時外側輪還會再加 ω·W/2）⇒ 要在 Nav2 側加限速。
  * 指令沒超標、量到的超標 ⇒ 韌體增量式 PID 的過衝（方向反轉時特別明顯）
    ⇒ 要在控制迴路或韌體側夾住，改 Nav2 沒有用。

只看其中一個就下結論，就是拿「現成的東西」冒充量測（memory: measurement-not-substitution）。

⚠ 驗收判準用的是**輪速**（packet EX4：FBK 輪速全程 ≤0.28 m/s）；
   **不可以看 EKF 估計值**——EKF 在加速段會高估，實測 0.470 vs 真實 0.321（2026-08-11）。

用法（在 Pi 上，bag 要已經收好）：
    python3 check_speed_limit.py <bag 目錄> [--limit 0.28]
"""
import argparse
import sys

LIMIT_MS = 0.28
"""packet EX4 的驗收上限（探索設定值 0.25 + 餘裕）。"""

TRACK_WIDTH_M = 0.14
"""輪距，用來從車體速度換算單輪速度：v_wheel = v ± ω·W/2。"""

BRIDGE_MAX_LINEAR = 0.5
BRIDGE_MAX_ANGULAR = 2.0
"""bridge_node 的預設夾限（max_linear／max_angular）。

⚠ **判主因要用「夾限後送到韌體的」指令，不能用 Nav2 原始 /cmd_vel**（2026-09-24）：
run5 原始指令換算單輪最高 0.440（ω 最高 4.1 rad/s），但 bridge 把 ω 夾到 2.0 後，
送到韌體的只有 0.290——拿原始值會把「韌體過衝」誤判成「Nav2 指令」，
我當天就據此下錯一次結論。
⚠ 本工具**不重現** bridge 的指令整形（CmdShaper，依時序而定），所以開了整形時，
「送到韌體」這一列是上界，實際送出的只會更小。
"""


def bridge_clamp(v, w):
    """與 bridge_node.cmd_cb 相同的夾限（含「低速不原地轉」）。"""
    v = max(-BRIDGE_MAX_LINEAR, min(BRIDGE_MAX_LINEAR, v))
    w = max(-BRIDGE_MAX_ANGULAR, min(BRIDGE_MAX_ANGULAR, w))
    if abs(v) < 0.01 and abs(w) > 0.01:
        w = 0.0
    return v, w

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def pct(n, d):
    return 100.0 * n / d if d else 0.0


def summarise(name, vals, limit):
    if not vals:
        print("  %-22s（沒有資料）" % name)
        return 0
    over = [v for v in vals if abs(v) > limit]
    print("  %-22s 樣本 %5d　最大 |v| %.3f　超過 %.2f 的有 %d 筆 (%.1f%%)"
          % (name, len(vals), max(abs(v) for v in vals), limit, len(over), pct(len(over), len(vals))))
    return len(over)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--limit", type=float, default=LIMIT_MS)
    a = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    cmd_body, cmd_wheel, fw_wheel, meas_wheel = [], [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/cmd_vel" and types.get(topic, "").endswith("Twist"):
            m = deserialize_message(data, Twist)
            v, w = m.linear.x, m.angular.z
            cmd_body.append(v)
            # 指令換算成單輪速度＝差速車公式，轉彎時外側輪會超過車體速度
            cmd_wheel.extend([v - w * TRACK_WIDTH_M / 2.0, v + w * TRACK_WIDTH_M / 2.0])
            fv, fw = bridge_clamp(v, w)
            fw_wheel.extend([fv - fw * TRACK_WIDTH_M / 2.0, fv + fw * TRACK_WIDTH_M / 2.0])
        elif topic == "/odom" and types.get(topic, "").endswith("Odometry"):
            m = deserialize_message(data, Odometry)
            v, w = m.twist.twist.linear.x, m.twist.twist.angular.z
            meas_wheel.extend([v - w * TRACK_WIDTH_M / 2.0, v + w * TRACK_WIDTH_M / 2.0])

    print("限速判準：單輪速度 ≤ %.2f m/s（packet EX4）" % a.limit)
    print()
    n_cmd_body = summarise("指令 車體速度", cmd_body, a.limit)
    n_cmd_wheel = summarise("指令 換算單輪", cmd_wheel, a.limit)
    n_fw_wheel = summarise("送到韌體（夾限後）", fw_wheel, a.limit)
    n_meas = summarise("實際 量到的輪速", meas_wheel, a.limit)
    print()

    if n_meas == 0:
        print("✅ PASS：實際輪速全程未超標")
        return 0

    print("❌ FAIL：實際輪速有 %d 筆超標" % n_meas)
    # ⚠ 兩個來源會疊加，不是二選一（2026-08-30 EX4 實測：指令 0.289／實際 0.487）。
    #   只報其中一個會把人導向錯誤的修法，所以兩個都判、並指出誰是主因。
    max_cmd_wheel = max((abs(v) for v in fw_wheel), default=0.0)
    max_meas = max((abs(v) for v in meas_wheel), default=0.0)
    overshoot = max_meas - max_cmd_wheel

    if n_fw_wheel > 0:
        print("   ● 指令側有貢獻：送到韌體的單輪最大 %.3f 已超過 %.2f"
              % (max_cmd_wheel, a.limit))
        print("     （`desired_linear_vel` 只是巡航速度不是上限；轉彎另加 ω·W/2＝%.3f m/s）"
              % (TRACK_WIDTH_M / 2.0))
    if overshoot > 0.05:
        print("   ● 控制迴路／韌體側有貢獻：實際比指令多 **%.3f m/s**（%.3f → %.3f）"
              % (overshoot, max_cmd_wheel, max_meas))
        print("     ＝增量式 PID 的過衝，**改 Nav2 參數無效**，要在控制迴路或韌體側夾")

    dominant = "控制迴路／韌體過衝" if overshoot > max(0.0, max_cmd_wheel - a.limit) else "Nav2 指令"
    print("   ⇒ 主因＝**%s**" % dominant)
    return 1


if __name__ == "__main__":
    sys.exit(main())
