"""光達安裝位姿 profile 的一致性鎖（RM1，2026-08-10）。

背景：`map_from_bag.sh` 舊版把靜態 TF 寫死成退役 X4 的 0.123/0.185/yaw=0。拿它重建
C1 錄的 bag，每一幀掃描都會投影錯位置——**地圖歪掉且完全不報錯**。修法＝安裝值移出腳本，
統一放 config/lidar_profiles.json，由本測試鎖住三件事：
  1. 三個型號都在，且值是實測值（改值必須同時改這裡，逼出一次人工確認）
  2. profiles.c1 與 bringup.launch.py 的預設值逐項相同（兩處不同步＝下一個 2026-07-20 事故）
  3. 未知型號必須 fail-closed（腳本拒跑），不得回退到任何預設值

（文字/JSON 比對，不實例化 launch：維持全套測試在無 ROS 的 Windows 上裸 pytest 可跑。）
"""
import json
import os
import re

HERE = os.path.dirname(__file__)
PROFILES = os.path.join(HERE, "..", "config", "lidar_profiles.json")
LAUNCH = os.path.join(HERE, "..", "launch", "bringup.launch.py")
SCRIPT = os.path.join(HERE, "..", "..", "..", "tools", "map_from_bag.sh")

# 實測值來源見 profile 檔的 measured 欄位與專案 CLAUDE.md
EXPECTED = {
    "c1":      {"laser_x": 0.19,  "laser_z": 0.155, "laser_yaw": 3.14159265},
    "rplidar": {"laser_x": 0.123, "laser_z": 0.185, "laser_yaw": 3.14159265},
    "ydlidar": {"laser_x": 0.123, "laser_z": 0.185, "laser_yaw": 0.0},
}


def _profiles():
    return json.load(open(PROFILES, encoding="utf-8"))["profiles"]


def test_three_lidar_profiles_present_with_measured_values():
    prof = _profiles()
    assert set(prof) == set(EXPECTED), (
        f"profile 型號集合變了：{sorted(prof)}，預期 {sorted(EXPECTED)}——"
        f"新增/移除光達要同步更新本測試與 bag_hint")
    for name, want in EXPECTED.items():
        for key, val in want.items():
            assert prof[name][key] == val, (
                f"{name}.{key}={prof[name][key]}，預期 {val}（實測值）——"
                f"改安裝位置要同時更新 profile、本測試、與該車的啟動說明")


def test_c1_profile_matches_bringup_defaults():
    """profile 的 c1 必須與 bringup 預設值一致——兩處不同步就是靜默歪圖的來源。"""
    src = open(LAUNCH, encoding="utf-8").read()
    defaults = {m.group(1): m.group(2) for m in re.finditer(
        r"DeclareLaunchArgument\(\s*'(\w+)'\s*,\s*default_value='([^']*)'", src)}
    c1 = _profiles()["c1"]
    for key in ("laser_x", "laser_z", "laser_yaw"):
        assert float(defaults[key]) == c1[key], (
            f"bringup 預設 {key}={defaults[key]} 與 lidar_profiles.json 的 c1 "
            f"{key}={c1[key]} 不一致——換光達時兩邊要一起改")


def test_every_profile_carries_provenance():
    for name, p in _profiles().items():
        assert p.get("measured"), f"{name} 缺 measured 欄位（值從哪來要留得下來）"
        assert p.get("desc"), f"{name} 缺 desc"


def test_bag_hints_reference_known_profiles():
    """既有 bag 的 profile 提示不得指向不存在的型號（選錯＝重建圖歪掉且不報錯）。"""
    spec = json.load(open(PROFILES, encoding="utf-8"))
    known = set(spec["profiles"])
    for bag, prof in spec["_bag_hint"].items():
        if bag.startswith("_"):
            continue
        assert prof in known, f"bag {bag} 指到不存在的 profile {prof!r}"


def test_script_has_no_hardcoded_laser_geometry():
    """腳本不得再自帶一份安裝值——重複定義就是下次不同步的來源。"""
    src = open(SCRIPT, encoding="utf-8").read()
    body = "\n".join(ln for ln in src.splitlines() if not ln.lstrip().startswith("#"))
    for bad in ("0.123", "0.185", "0.19", "0.155"):
        assert bad not in body, (
            f"map_from_bag.sh 的程式碼裡出現寫死的安裝值 {bad}——"
            f"安裝值只能來自 lidar_profiles.json")


def test_script_requires_lidar_and_fails_closed():
    src = open(SCRIPT, encoding="utf-8").read()
    assert '[ -n "$LIDAR" ] || die' in src, "--lidar 必須是必填，不得有預設值"
    assert "未知光達型號" in src, "未知型號必須明確拒跑"
    assert "lidar_profiles.json" in src, "安裝值必須讀 profile 檔"
