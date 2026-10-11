"""EX1「建圖中導航」分支的結構鎖（2026-08-11）。

要防的事只有一件：**map->odom 這個 TF 只能有一個發布者**。
slam:=false 由 AMCL 發、slam:=true 由 slam_toolbox 發；兩邊同時起會互相蓋掉，
而且 TF 打架不會報錯，只會讓地圖與導航靜默歪掉（專案地雷 2026-07-20 同款症狀）。
所以本測試機械檢查三件事：
  1. 三個定位相關節點的啟動條件互斥（amcl/map_server＝Unless(slam)、slam_toolbox＝If(slam)）；
  2. slam 模式的 lifecycle_manager 不去管 map_server/amcl（管了會等不到節點而整批卡住）；
  3. slam 預設 false——加這個分支不准動到現行「用既有地圖導航」的行為。

用 ast 解析而非字串比對：condition 是結構而不是文字，regex 抓不準。
（Windows 測試機無 ROS，全套測試維持裸 pytest 可跑，故不實例化 LaunchDescription。）
"""
import ast
import os

import yaml

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, "..", "launch", "nav2.launch.py")
SLAM_PARAMS = os.path.join(HERE, "..", "config", "slam_toolbox_mapping.yaml")


def _kw(call, name):
    for k in call.keywords:
        if k.arg == name:
            return k.value
    return None


def _const_str(node):
    return node.value if isinstance(node, ast.Constant) else None


def _list_vars():
    """解析 launch 檔裡的字串清單變數，支援 `a = [...]` 與 `b = [...] + a`。"""
    tree = ast.parse(open(LAUNCH, encoding="utf-8").read())
    out = {}

    def value_of(node):
        if isinstance(node, ast.List):
            return [_const_str(e) for e in node.elts]
        if isinstance(node, ast.Name):
            return out.get(node.id)
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
            left, right = value_of(node.left), value_of(node.right)
            return None if left is None or right is None else left + right
        return None

    for stmt in ast.walk(tree):
        if isinstance(stmt, ast.Assign) and len(stmt.targets) == 1 \
                and isinstance(stmt.targets[0], ast.Name):
            v = value_of(stmt.value)
            if v is not None and all(isinstance(x, str) for x in v):
                out[stmt.targets[0].id] = v
    return out


def _nodes():
    """回傳 {節點名: {'kind': 'Node'|'LifecycleNode', 'cond': 'IfCondition'|'UnlessCondition'|None,
                      'cond_arg': str|None, 'call': ast.Call}}；同名多筆存成 list。"""
    tree = ast.parse(open(LAUNCH, encoding="utf-8").read())
    found = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        fn = call.func.id if isinstance(call.func, ast.Name) else None
        if fn not in ("Node", "LifecycleNode"):
            continue
        name = _const_str(_kw(call, "name"))
        cond = _kw(call, "condition")
        kind = cond_arg = None
        if isinstance(cond, ast.Call) and isinstance(cond.func, ast.Name):
            kind = cond.func.id
            # IfCondition(slam) 或 IfCondition(AndSubstitution(autostart, slam))
            names = [n.id for n in ast.walk(cond) if isinstance(n, ast.Name)]
            cond_arg = "slam" if "slam" in names else ",".join(names)
        found.setdefault(name, []).append(
            {"kind": fn, "cond": kind, "cond_arg": cond_arg, "call": call})
    return found


def test_slam_defaults_to_false():
    """預設必須維持舊行為（靜態地圖＋AMCL）——EX1 是加分支，不是換掉現行跑法。"""
    src = open(LAUNCH, encoding="utf-8").read()
    tree = ast.parse(src)
    defaults = {}
    for call in ast.walk(tree):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == "DeclareLaunchArgument" and call.args):
            key = _const_str(call.args[0])
            defaults[key] = _const_str(_kw(call, "default_value"))
    assert defaults.get("slam") == "false", \
        f"slam 預設應為 'false'，實際 {defaults.get('slam')!r}"
    assert defaults.get("slam_params_file") is None, \
        "slam_params_file 的預設值應是組出來的路徑（非字面字串），這裡抓到字面值代表寫死了"


def test_localization_and_slam_are_mutually_exclusive():
    """AMCL/map_server 與 slam_toolbox 不得同時啟動——兩邊都會發 map->odom。"""
    n = _nodes()
    for name in ("map_server", "amcl"):
        entries = n.get(name)
        assert entries and len(entries) == 1, f"{name} 應恰好宣告一次，實際 {entries}"
        e = entries[0]
        assert e["cond"] == "UnlessCondition" and e["cond_arg"] == "slam", \
            f"{name} 必須是 UnlessCondition(slam)，實際 {e['cond']}({e['cond_arg']})"

    st = n.get("slam_toolbox")
    assert st and len(st) == 1, f"slam_toolbox 應恰好宣告一次，實際 {st}"
    assert st[0]["kind"] == "LifecycleNode", \
        "slam_toolbox 在 Jazzy 是 lifecycle node，必須用 LifecycleNode 宣告才 configure/activate 得動"
    assert st[0]["cond"] == "IfCondition" and st[0]["cond_arg"] == "slam", \
        f"slam_toolbox 必須是 IfCondition(slam)，實際 {st[0]['cond']}({st[0]['cond_arg']})"


def test_slam_lifecycle_manager_does_not_manage_amcl():
    """slam 模式的 lifecycle_manager 若還列 map_server/amcl，會等不到節點而整批卡在 configuring。"""
    mgrs = _nodes().get("lifecycle_manager_nav")
    assert mgrs and len(mgrs) == 2, \
        f"應有兩個 lifecycle_manager_nav（slam 與非 slam 各一），實際 {len(mgrs) if mgrs else 0} 個"

    by_cond = {m["cond"]: m for m in mgrs}
    assert set(by_cond) == {"IfCondition", "UnlessCondition"}, \
        f"兩個 lifecycle_manager 的條件必須互斥，實際 {[m['cond'] for m in mgrs]}"

    lists = _list_vars()

    def node_names(call):
        """取出 parameters 裡 'node_names' 對到的值；它是變數名，回去查該變數的清單內容。"""
        params = _kw(call, "parameters")
        for d in ast.walk(params):
            if not isinstance(d, ast.Dict):
                continue
            for k, v in zip(d.keys, d.values):
                if _const_str(k) == "node_names":
                    if isinstance(v, ast.Name):
                        assert v.id in lists, f"找不到變數 {v.id} 的清單定義"
                        return lists[v.id]
                    return [_const_str(x) for x in getattr(v, "elts", [])]
        raise AssertionError("lifecycle_manager 的 parameters 裡找不到 node_names")

    slam_names = node_names(by_cond["IfCondition"]["call"])
    assert "amcl" not in slam_names and "map_server" not in slam_names, \
        f"slam 模式不得管理 amcl/map_server（它們根本沒起），實際 node_names 含 {slam_names}"
    assert "controller_server" in slam_names and "bt_navigator" in slam_names, \
        f"slam 模式仍須管理導航節點，實際 {slam_names}"

    loc_names = node_names(by_cond["UnlessCondition"]["call"])
    assert "amcl" in loc_names and "map_server" in loc_names, \
        f"非 slam 模式必須管理 amcl/map_server，實際 {loc_names}"


def test_slam_params_use_this_car_tf_frames():
    """base_frame 寫錯＝slam_toolbox 查不到 TF、完全建不了圖（本車沒有 base_footprint）。"""
    with open(SLAM_PARAMS, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    assert "/**" in cfg, \
        "頂層 key 必須是萬用 '/**'——寫死節點名一進 namespace 就整份靜默失效（地雷 2026-07-10）"
    p = cfg["/**"]["ros__parameters"]
    assert p["base_frame"] == "base_link", \
        f"base_frame 應為 base_link（本車 TF 樹沒有 base_footprint），實際 {p['base_frame']!r}"
    assert p["odom_frame"] == "odom" and p["map_frame"] == "map"
    assert p["scan_topic"] == "/scan"
    assert p["mode"] == "mapping", "EX1 要的是建圖模式，不是 localization"


def test_slam_params_cover_the_two_warnings_seen_in_ex1_field_test():
    """2026-08-11 EX1 實跑時 slam_toolbox 自己抱怨的兩項，別再被拿掉。"""
    with open(SLAM_PARAMS, encoding="utf-8") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    assert p["max_laser_range"] == 16.0, \
        "max_laser_range 必須＝C1 實際量程 16 m；內建預設 25 m 會讓 slam 啟動即 WARN"
    assert p["stack_size_to_use"] == 40000000, \
        "少了 stack_size_to_use，slam_toolbox 每 15 秒 WARN 一次，且 RM 線存姿態圖會踩到"
