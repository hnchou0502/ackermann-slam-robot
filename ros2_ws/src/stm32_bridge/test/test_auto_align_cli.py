"""auto_align_maps CLI 行為測試（Codex R4-F2：exit code 政策要被咬住）。

自產微型 PGM/yaml fixture（不進 repo 大檔），驗證：
  - 歧義幾何（直牆）→ 防護攔下 → exit 3、無 TF 輸出
  - 同案例加 --force → exit 0、有 TF
  - 唯一幾何（L 形）→ exit 0、有 TF
"""
import os
import subprocess
import sys

import numpy as np
import pytest

TOOL = os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools",
                    "auto_align_maps.py")
PKG_PARENT = os.path.join(os.path.dirname(__file__), "..")


def write_map(prefix, occ_rc, w, h, res=0.05, origin=(0.0, 0.0)):
    """寫最小 PGM(P5)+yaml。occ_rc＝牆格 (row, col) 清單（PGM row 0 在頂）。"""
    arr = np.full((h, w), 254, dtype=np.uint8)
    for r, c in occ_rc:
        arr[r, c] = 0
    with open(prefix + ".pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(arr.tobytes())
    with open(prefix + ".yaml", "w", encoding="utf-8") as f:
        f.write(f"image: {os.path.basename(prefix)}.pgm\nmode: trinary\n"
                f"resolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")


def run_tool(*args):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.abspath(PKG_PARENT) + os.pathsep + env.get("PYTHONPATH", "")
    env["PYTHONIOENCODING"] = "utf-8"
    return subprocess.run([sys.executable, TOOL, *args],
                          capture_output=True, text=True, encoding="utf-8",
                          env=env, timeout=600)


@pytest.fixture
def line_pair(tmp_path):
    # 3m 直牆（60 格），A/B 相同 → 0°/180° 與沿牆平移皆歧義
    a = str(tmp_path / "line_a")
    b = str(tmp_path / "line_b")
    cells = [(2, c) for c in range(2, 62)]
    write_map(a, cells, w=64, h=5)
    write_map(b, cells, w=64, h=5)
    return a, b


@pytest.fixture
def l_pair(tmp_path):
    # L 形唯一幾何（2m × 1.5m 外框缺一角）
    a = str(tmp_path / "l_a")
    b = str(tmp_path / "l_b")
    cells = set()
    for c in range(0, 40):
        cells.add((0, c))
        if c <= 20:
            cells.add((30, c))
    for r in range(0, 31):
        cells.add((r, 0))
        if r >= 15:
            cells.add((r, 20))
    for c in range(20, 40):
        cells.add((15, c))
    for r in range(0, 16):
        cells.add((r, 39))
    a_cells = sorted(cells)
    write_map(a, a_cells, w=40, h=31)
    write_map(b, a_cells, w=40, h=31)
    return a, b


def test_ambiguous_geometry_exits_3_without_tf(line_pair):
    p = run_tool(*line_pair)
    assert p.returncode == 3, f"stdout={p.stdout}\nstderr={p.stderr}"
    assert "\nTF " not in p.stdout


def test_force_overrides_guard(line_pair):
    p = run_tool(*line_pair, "--force")
    assert p.returncode == 0, f"stdout={p.stdout}\nstderr={p.stderr}"
    assert "\nTF " in p.stdout


def test_unique_geometry_exits_0_with_tf(l_pair):
    p = run_tool(*l_pair)
    assert p.returncode == 0, f"stdout={p.stdout}\nstderr={p.stderr}"
    assert "\nTF " in p.stdout
