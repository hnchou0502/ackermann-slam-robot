#!/usr/bin/env python3
"""RM2／RM3 驗收條件①：證明第二遍真的接上了第一遍的姿態圖。

判定邏輯全在零 ROS 的 `stm32_bridge/posegraph_math.py`（筆電可 pytest）；
這支只負責接 ROS 的線與檔案 I/O，照 follow_math／explore_math 的分工慣例。

## 怎麼用（RM2 現場三步）

1. **第一遍跑完、存姿態圖的當下**（⚠ 一定要在第二遍開跑之前）：

       python3 check_posegraph_continuity.py snapshot --out ~/rm2_baseline.json

2. 車放回地板膠帶標記點，載入第一遍姿態圖跑第二遍。

3. **第二遍跑完、slam_toolbox 還活著時**：

       python3 check_posegraph_continuity.py verify --baseline ~/rm2_baseline.json \
           --out ~/rm2_final.json

   判決 PASS ⇒ exit 0；FAIL／INVALID ⇒ exit 1。兩個 json 都留著當驗收證據。

事後要重驗（或在筆電上重看）不必再碰車：

       python3 check_posegraph_continuity.py verify --baseline b.json --offline f.json

## ⚠ 步驟 1 的時點是這支工具唯一無法自我保護的地方

baseline 若在第二遍已經開跑後才取，混進去的第二遍節點會讓判定**假通過**
（詳見 posegraph_math 檔頭）。這件事無法從資料本身檢查出來，只能靠操作順序保證。
"""
import argparse
import json
import sys
import time

# 這支要能在沒有 ROS 的筆電上跑 offline 模式，所以 rclpy 一律延後 import
from stm32_bridge.posegraph_math import PASS, analyse, format_report, resolve_edges

GRAPH_TOPIC = "/slam_toolbox/graph_visualization"
NODE_NS = "slam_toolbox"
EDGE_NS = "slam_toolbox_edges"
SPHERE, LINE_LIST = 2, 5


def capture_from_ros(timeout_s):
    """訂閱 graph_visualization，抓一幀完整的姿態圖。

    節點：ns='slam_toolbox' 的 SPHERE，marker.id ＝節點 ID、pose ＝位置。
    邊　：ns='slam_toolbox_edges' 的 LINE_LIST，points 兩兩一組＝一條邊（只有座標沒有 ID）。
    這兩件事是 2026-08-29 在 car1 上實測確認的，不是照文件寫的。
    """
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import MarkerArray

    class Capture(Node):
        def __init__(self):
            super().__init__("posegraph_capture")
            self.create_subscription(MarkerArray, GRAPH_TOPIC, self.cb, 10)
            self.data = None

        def cb(self, msg):
            if self.data is not None:
                return
            nodes, edge_points = [], []
            for m in msg.markers:
                if m.ns == NODE_NS and m.type == SPHERE:
                    nodes.append([int(m.id), float(m.pose.position.x), float(m.pose.position.y)])
                elif m.ns == EDGE_NS and m.type == LINE_LIST:
                    p = m.points
                    for i in range(0, len(p) - 1, 2):
                        edge_points.append([float(p[i].x), float(p[i].y),
                                            float(p[i + 1].x), float(p[i + 1].y)])
            self.data = {"nodes": sorted(nodes), "edge_points": edge_points}

    rclpy.init()
    try:
        n = Capture()
        t0 = time.time()
        while rclpy.ok() and n.data is None and time.time() - t0 < timeout_s:
            rclpy.spin_once(n, timeout_sec=0.5)
        data = n.data
    finally:
        rclpy.shutdown()
    if data is None:
        raise SystemExit("在 %.0f 秒內收不到 %s——slam_toolbox 沒在跑，或還沒 activate"
                         % (timeout_s, GRAPH_TOPIC))
    return data


def load_dump(path):
    with open(path) as f:
        d = json.load(f)
    return {"nodes": d["nodes"], "edge_points": d["edge_points"]}


def as_node_dict(dump):
    return {int(n[0]): (float(n[1]), float(n[2])) for n in dump["nodes"]}


def save(dump, path):
    if path:
        with open(path, "w") as f:
            json.dump(dump, f)
        print("已存 %s（節點 %d／線段 %d）" % (path, len(dump["nodes"]), len(dump["edge_points"])))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("snapshot", help="記下第一遍結束時的姿態圖（第二遍開跑前執行）")
    s.add_argument("--out", required=True)
    s.add_argument("--timeout", type=float, default=30.0)

    v = sub.add_parser("verify", help="比對第二遍的姿態圖，判定有無跨遍約束")
    v.add_argument("--baseline", required=True, help="snapshot 產出的 json")
    v.add_argument("--offline", help="不連 ROS，改讀這個已存的最終圖 json")
    v.add_argument("--out", help="把最終圖也存一份當證據")
    v.add_argument("--timeout", type=float, default=30.0)

    a = ap.parse_args(argv)

    if a.cmd == "snapshot":
        dump = capture_from_ros(a.timeout)
        save(dump, a.out)
        nodes = as_node_dict(dump)
        print("baseline 節點 ID：%s" % sorted(nodes))
        if not nodes:
            print("⚠ 抓到 0 個節點——第一遍根本沒建出東西，別拿它當 baseline")
            return 1
        return 0

    base = as_node_dict(load_dump(a.baseline))
    final_dump = load_dump(a.offline) if a.offline else capture_from_ros(a.timeout)
    save(final_dump, a.out)

    final_nodes = as_node_dict(final_dump)
    edges, problems = resolve_edges(final_nodes, final_dump["edge_points"])
    if problems:
        # 端點對不回節點＝圖抓得不完整，這時任何分類結果都不可信，直接擋下
        print("判決：INVALID")
        print("理由：邊的端點無法明確對回節點，共 %d 筆問題" % len(problems))
        for p in problems[:5]:
            print("  - %s" % p)
        return 1

    res = analyse(base, final_nodes, edges)
    print(format_report(res))
    return 0 if res["verdict"] == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
