"""launch 預設值一致性鎖（Codex F1/F4，2026-07-21）。

背景：car1 換 C1 時只改了 laser_x/z/yaw 預設、沒改 lidar_type 預設（仍指退役的 X4），
造成「裸跑 bringup＝X4 驅動配 C1 的 TF」的自相矛盾組合——地圖會靜默歪掉。
本測試以文字層面鎖住「預設驅動與預設 TF 必須屬同一硬體（目前＝car1 的 C1）」。
改任何一邊而沒改另一邊，這裡會紅。

（文字比對而非 launch 實例化：Windows 測試機無 ROS，維持全套測試裸 pytest 可跑。）
"""
import os
import re

LAUNCH = os.path.join(os.path.dirname(__file__), "..", "launch", "bringup.launch.py")

# car1 現役硬體＝RPLIDAR C1 的一致組合（2026-07-20 實測值）
EXPECTED = {
    "lidar_type": "c1",
    "laser_x": "0.19",
    "laser_z": "0.155",
    "laser_yaw": "3.14159265",
}


def _defaults():
    src = open(LAUNCH, encoding="utf-8").read()
    out = {}
    for m in re.finditer(
            r"DeclareLaunchArgument\(\s*'(\w+)'\s*,\s*default_value='([^']*)'", src):
        out[m.group(1)] = m.group(2)
    return out


def test_default_lidar_and_tf_belong_to_same_hardware():
    d = _defaults()
    for key, want in EXPECTED.items():
        assert d.get(key) == want, (
            f"預設 {key}={d.get(key)!r}，預期 {want!r}——"
            f"預設驅動與預設 TF 必須同屬一套硬體（見 Codex F1）；"
            f"換光達時四個值要一起改，並更新本測試")


def test_c1_branch_exists_and_uses_sllidar():
    src = open(LAUNCH, encoding="utf-8").read()
    assert "sllidar_ros2" in src and "'c1'" in src, \
        "c1 分支（sllidar_ros2）不見了——C1 是 car1 現役光達"
