#!/usr/bin/env python3
"""即時雙車合圖節點（CM2 架構的合圖端）。

**背景（2026-07-21 實查後的路線修正）**：CM2 原規劃的
`decentralized_multirobot_slam_toolbox_node` 在 ROS2 主線**不存在**（僅 ros1_multirobot
老分支；apt 與 source build 內容相同、六個節點都有）。故 CM2 改走：
兩車各自跑 slam_toolbox（既有能力）→ 本節點訂閱兩張 /map 即時融合出一張全域圖。

**演算法與數學全在 `merge_math.py`**（零 ROS 依賴、Windows 可 pytest——Codex F4 重構）。
對齊成功後快取變換，之後每次地圖更新只重合成不重搜尋；週期性重對齊吸收漂移。

**發布前三道防護（Codex F3）**，任一不過＝不發布並 log 原因（防垃圾合圖）：
  1. score >= min_score          （B 牆落在 A 牆上的比例）
  2. ambiguity <= max_ambiguity  （與最佳角相距 ≥15° 的第二峰分數比；直牆/重複走廊的解毒劑）
  3. conflict <= max_conflict    （B 牆落在 A「明確空曠格」的比例；錯位對齊的解毒劑）
  另 saturated（近滿分角過多＝評分飽和）一律拒絕。

**桌測劇本（不用開車）**：car2 起 map_server 發 /car2/map，car1 起本節點跨 WiFi 訂閱
＋發布 /merged_map——即為 CM2 減去「行進中建圖」的全架構煙測（2026-07-21 已通過）。

參數：
  topic_a / topic_b     兩張輸入圖（預設 /car1/map、/car2/map）；A 系＝輸出圖座標系
  align_period_sec      重新對齊週期（預設 60；<=0 表示只對齊一次）
  min_score             對齊分數門檻（預設 0.30）
  max_ambiguity         歧義比門檻（預設 0.80，實資料校準：正解≤0.53、真歧義 0.896）
  max_conflict          空曠衝突比門檻（預設 0.25，實資料校準：正解≤0.198、錯解≥0.330）
  coarse_deg            粗掃角度步長（預設 2°）
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

from stm32_bridge import merge_math as mm


def _grid_dict(msg):
    return dict(data=msg.data, width=msg.info.width, height=msg.info.height,
                res=msg.info.resolution,
                ox=msg.info.origin.position.x, oy=msg.info.origin.position.y)


class MapMergeLive(Node):
    def __init__(self):
        super().__init__("map_merge_live")
        self.declare_parameter("topic_a", "/car1/map")
        self.declare_parameter("topic_b", "/car2/map")
        self.declare_parameter("align_period_sec", 60.0)
        # 門檻預設值住在 merge_math（政策唯一真相源，test_merge_math 鎖常數；
        # 校準依據見 merge_math 常數區註解與 docs/design/merge_guard_calibration.md）。
        self.declare_parameter("min_score", mm.DEFAULT_MIN_SCORE)
        self.declare_parameter("max_ambiguity", mm.DEFAULT_MAX_AMBIGUITY)
        self.declare_parameter("max_conflict", mm.DEFAULT_MAX_CONFLICT)
        self.declare_parameter("coarse_deg", 2.0)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.map_a = None
        self.map_b = None
        self.tf = None          # (yaw, tx, ty)
        self.lock = threading.Lock()
        self.aligning = False

        ta = self.get_parameter("topic_a").value
        tb = self.get_parameter("topic_b").value
        self.create_subscription(OccupancyGrid, ta, self.cb_a, qos)
        self.create_subscription(OccupancyGrid, tb, self.cb_b, qos)
        self.pub = self.create_publisher(OccupancyGrid, "merged_map", qos)
        self.create_timer(5.0, self.tick)
        self.last_align_t = 0.0
        self.get_logger().info(f"訂閱 A={ta} B={tb}，等兩張圖到齊…")

    def cb_a(self, msg):
        with self.lock:
            self.map_a = msg

    def cb_b(self, msg):
        with self.lock:
            self.map_b = msg

    def tick(self):
        with self.lock:
            a, b = self.map_a, self.map_b
            if a is None or b is None or self.aligning:
                return
            now = self.get_clock().now().nanoseconds / 1e9
            period = float(self.get_parameter("align_period_sec").value)
            need_align = self.tf is None or (period > 0 and now - self.last_align_t > period)
            if need_align:
                self.aligning = True   # 在 lock 內設旗標，防兩個 timer tick 重入
        if need_align:
            threading.Thread(target=self.do_align, args=(a, b), daemon=True).start()
        elif self.tf is not None:
            self.publish_merged(a, b)

    def do_align(self, a, b):
        try:
            t0 = self.get_clock().now().nanoseconds / 1e9
            res = a.info.resolution
            if abs(res - b.info.resolution) > 1e-9:
                self.get_logger().error("兩圖解析度不同，無法合併")
                return
            ga, gb = _grid_dict(a), _grid_dict(b)
            pa = mm.walls_from_grid(**ga)
            pb = mm.walls_from_grid(**gb)
            if len(pa) < 20 or len(pb) < 20:
                self.get_logger().warning(f"牆格太少（A={len(pa)} B={len(pb)}），先不對齊")
                return
            out = mm.align(pa, pb, res, float(self.get_parameter("coarse_deg").value))
            dt = self.get_clock().now().nanoseconds / 1e9 - t0
            if out is None:
                self.get_logger().error("對齊失敗（無有效角度）")
                return

            score, yaw, tx, ty = out["score"], out["yaw"], out["tx"], out["ty"]
            conflict = mm.free_conflict_ratio(
                mm.free_from_grid(**ga), pb, tx, ty, yaw, res)
            diag = (f"yaw {math.degrees(yaw):+.1f}° t=({tx:+.2f},{ty:+.2f}) "
                    f"分數 {score:.2f} 歧義比 {out['ambiguity']:.2f}"
                    f"({out.get('runner_kind','-')}) 空曠衝突 {conflict:.2f} 耗時 {dt:.1f}s")

            # 決策交給 merge_math.accept（政策唯一實作，可測；Codex R4-F2）
            ok, reason = mm.accept(
                out, conflict,
                min_score=float(self.get_parameter("min_score").value),
                max_ambiguity=float(self.get_parameter("max_ambiguity").value),
                max_conflict=float(self.get_parameter("max_conflict").value))
            if not ok:
                self.get_logger().warning(f"拒絕（{reason}）——{diag}")
                return

            self.tf = (yaw, tx, ty)
            self.last_align_t = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"對齊 OK：{diag}")
            self.publish_merged(a, b)
        finally:
            self.aligning = False

    def publish_merged(self, a, b):
        yaw, tx, ty = self.tf
        out, minx, miny = mm.fuse_grids(_grid_dict(a), _grid_dict(b), tx, ty, yaw)
        H, W = out.shape

        msg = OccupancyGrid()
        msg.header.frame_id = a.header.frame_id or "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = a.info.resolution
        msg.info.width = W
        msg.info.height = H
        msg.info.origin.position.x = minx
        msg.info.origin.position.y = miny
        msg.info.origin.orientation.w = 1.0
        msg.data = out.flatten().tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MapMergeLive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
