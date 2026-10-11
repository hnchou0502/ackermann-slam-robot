#!/usr/bin/env python3
"""EX4／RM3 開跑前的 preflight 硬 gate：任一項不成立就不准啟動自主探索。

packet EX4 步驟①要求「五項故障注入證據逐名檢查全綠，缺任一份＝不准跑，
尤其不得省略 supervisor SIGKILL」。人工逐條看容易漏，所以機器化。

檢查分兩類：
  **可機器驗的**（本腳本自動判定）：故障注入證據、陀螺零漂、節點殘留、限速設定
  **只能人確認的**（本腳本列出來要求逐條回答 yes）：場地落差、低矮障礙、車擺放位置、人在場

在 Pi 上跑（要驗陀螺零漂就必須 bringup 正在跑）：
    python3 ex4_preflight.py              # 完整檢查（互動）
    python3 ex4_preflight.py --no-human   # 只跑機器可驗的部分（給 CI／預檢用）

退出碼：0＝全過可以跑；1＝有項目未過，禁止啟動。
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def _find_up(*parts):
    """往上層目錄找檔案。repo 是 <repo>/ros2_ws/tools，但 Pi 上部署成 ~/ros2_ws/tools，
    層數不同——寫死相對層數會在其中一邊失效（2026-08-11 踩過）。"""
    for up in range(0, 4):
        cand = os.path.join(HERE, *(['..'] * up), *parts)
        if os.path.exists(cand):
            return os.path.abspath(cand)
    return os.path.join(HERE, '..', '..', *parts)


EVIDENCE = _find_up('docs', 'design', 'ex3_fault_injection.md')
EXPLORE_PARAMS = _find_up('src', 'stm32_bridge', 'config', 'nav2_params_explore.yaml')

# packet 逐名點的五項，缺一不可
REQUIRED_INJECTIONS = [
    ('1-ctrl-c', 'Ctrl-C supervisor'),
    ('2-kill-explorer', 'kill -9 explorer'),
    ('3-kill-controller', 'kill -9 controller_server'),
    ('4-kill-supervisor', 'kill -9 supervisor 本體（pdeathsig，最容易被省略的一項）'),
    ('5-unplug-usb', '拔 STM32 USB 線（硬體兜底）'),
]

GYRO_LIMIT_DEG_S = 0.2
EXPLORE_SPEED = 0.25

HUMAN_CHECKS = [
    '場地**沒有落差或樓梯口**，或所有邊緣已用實體護欄封死'
    '（護欄不可穿越且高於光達掃描面 0.155 m）——落差清不掉，只能不存在或封住',
    '低矮障礙已清空（光達掃描面在 0.155 m，以下的東西完全看不到：拖鞋、電線、寵物碗）',
    '**車鑽得進去的家具已移開或封住底部**（椅子、桌子、沙發底）——'
    '桌面/椅面在掃描面「上方」同樣看不見，車只看到細細的腳、把腳與腳之間判成可通行，'
    '就會鑽進去卡死（2026-08-30 EX4 首跑實際發生：車自己開到椅子下，'
    '之後每次規劃都 no valid path found，8 個目標全進黑名單、探索提早收工）',
    '車擺在**周圍約 1.5 m 內無遮擋**的開闊處（起步可見 ≥5 m²；牆角/沙發縫隙會讓探索跨不出第一步）',
    '車就定位後已按 STM32 RESET 並靜置 3 秒（搬完車才按，不是按完才搬）',
    '人全程在場，手放在 supervisor 終端上（終端標題設為 EXPLORE-SUPERVISOR）',
    '知道急停順序：先 Ctrl-C supervisor →（無效時）拔 STM32 USB 線',
    '寵物／小孩不在場地內',
]


def ok(msg):
    print('  [ OK ] %s' % msg)
    return True


def fail(msg):
    print('  [FAIL] %s' % msg)
    return False


def check_injection_evidence():
    print('1. 故障注入證據（packet EX4 硬 gate）')
    if not os.path.exists(EVIDENCE):
        return fail('找不到證據檔 %s' % EVIDENCE)
    text = open(EVIDENCE, encoding='utf-8').read()
    good = True
    for key, desc in REQUIRED_INJECTIONS:
        m = re.search(r'RESULT:\s*%s\s+(\w+)' % re.escape(key), text)
        if not m:
            good = fail('缺少 %s（%s）的結果標記' % (key, desc))
        elif m.group(1) != 'PASS':
            good = fail('%s（%s）＝%s' % (key, desc, m.group(1)))
        else:
            ok('%s：%s' % (key, desc))
    return good


def check_speed_limit():
    print('2. 探索限速')
    if not os.path.exists(EXPLORE_PARAMS):
        return fail('找不到 %s' % EXPLORE_PARAMS)
    m = re.search(r'desired_linear_vel:\s*([0-9.]+)',
                  open(EXPLORE_PARAMS, encoding='utf-8').read())
    if not m:
        return fail('參數檔裡找不到 desired_linear_vel')
    v = float(m.group(1))
    if abs(v - EXPLORE_SPEED) > 1e-6:
        return fail('探索限速是 %.2f，應為 %.2f m/s' % (v, EXPLORE_SPEED))
    return ok('限速 %.2f m/s（EX4 驗收時以 **FBK 輪速** ≤0.28 為準，不可看 EKF 估計值）' % v)


def _run(cmd, timeout=20):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout)
        return r.stdout + r.stderr
    except Exception as e:
        return 'ERROR %s' % e


def _count(pattern):
    out = _run("pgrep -f '%s' | wc -l" % pattern)
    try:
        return int(out.strip().splitlines()[-1])
    except Exception:
        return -1


def _count_movers():
    """會發 /cmd_vel 的節點數（Nav2 堆疊＋explorer）。"""
    return _count("^/opt/ros/jazzy/lib/nav2_|^/usr/bin/python3 .*/frontier_explorer")


def _count_bridges():
    return _count("^/usr/bin/python3 .*/stm32_bridge/bridge")


def check_no_leftover_nodes():
    """殘留檢查（Codex F4，2026-08-11 修正）。

    舊版把 bridge 也算進「殘留」，於是與下一項「量陀螺零漂需要 bringup 在跑」直接
    互相矛盾——完整 gate 永遠不可能全過（而我只跑過 --no-human 且沒起 bringup，
    所以沒發現）。

    正確的區分：
      - **會發 /cmd_vel 的節點（Nav2／explorer）必須是 0**：上一輪沒收乾淨的話，
        新舊兩套會同時對同一台車下令。
      - **bridge 必須是 0**（Codex R2-F2，2026-08-11 收緊）：放行後要啟動的
        `explore.launch.py` **自己還會再起一個 bridge**，既存那個會跟它搶 serial 埠；
        而且既存 bridge 不在 supervisor 的 pdeathsig 安全鏈裡，出事時不會被一起收掉。
        陀螺量測所需的唯讀 bringup 一律由 preflight 自己啟動、自己關閉。
    """
    print('3. 殘留節點檢查')
    movers = _count_movers()
    bridges = _count_bridges()
    if movers < 0 or bridges < 0:
        return fail('無法判定行程數（pgrep 不可用？）')
    good = True
    if movers != 0:
        good = fail('還有 %d 個會發 /cmd_vel 的節點在跑（Nav2／explorer），先收乾淨' % movers)
    else:
        ok('無殘留的 Nav2／explorer')
    if bridges != 0:
        good = fail('有 %d 個 bridge 在跑 → 先關掉。放行後 explore.launch.py 會自己起一個，'
                    '既存的會搶 serial 埠，且不在 supervisor 的安全鏈內' % bridges)
    else:
        ok('無殘留 bridge（陀螺量測會由 preflight 自己起唯讀版並關閉）')
    return good


def _start_readonly_bringup():
    """受控啟動**唯讀** bringup：enable_cmd:=false ＝絕不送 $CMD，車不可能動。

    這是 Codex F4 的建議做法——preflight 自己把量測環境拉起來、量完可靠關閉，
    而不是要求使用者先起一套（那會與殘留檢查打架，且那套之後還要關掉才能起 supervisor）。
    """
    subprocess.Popen(
        'setsid nohup ros2 launch stm32_bridge bringup.launch.py enable_cmd:=false '
        '>/tmp/preflight_bringup.log 2>&1 </dev/null &',
        shell=True)
    for _ in range(30):
        time.sleep(1)
        if _count_bridges() >= 1:
            time.sleep(6)          # 等 IMU 資料開始流動
            return True
    return False


def _stop_readonly_bringup():
    _run("pkill -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch stm32_bridge bringup'")
    _run("pkill -f '^/usr/bin/python3 .*/stm32_bridge/bridge'")
    _run("pkill -f '^/opt/ros/jazzy/lib/robot_localization/ekf_node'")
    _run("pkill -f '^/home/.*/sllidar_ros2/sllidar_node'")
    time.sleep(3)
    return _count_bridges() == 0


def check_gyro_bias():
    print('4. 陀螺零漂')
    script = os.path.join(HERE, 'gyro_check.py')
    if not os.path.exists(script):
        return fail('找不到 %s（工具應隨 repo 部署，不要放 /tmp）' % script)

    # 一律由 preflight 自己啟動／擁有／關閉唯讀 bringup（Codex R2-F2）：
    # 不接受「沿用既存 bridge」，那個 bridge 不在我們的掌控與安全鏈內。
    if _count_bridges() != 0:
        return fail('量陀螺前必須沒有其他 bridge 在跑（見上一項）')
    print('       （preflight 自己起唯讀 bringup，enable_cmd:=false，車不會動）')
    if not _start_readonly_bringup():
        _stop_readonly_bringup()
        return fail('唯讀 bringup 起不來，看 /tmp/preflight_bringup.log')

    out = _run('python3 %s' % script, timeout=40)
    if not _stop_readonly_bringup():
        return fail('唯讀 bringup 關不乾淨——殘留的 bridge 會讓 supervisor 搶不到 serial 埠')
    print('       （量完已關閉唯讀 bringup）')
    if 'ModuleNotFoundError' in out or 'No module named' in out:
        return fail('這個 shell 沒有 ROS 環境 → 先跑：'
                    ' source /opt/ros/jazzy/setup.bash && '
                    'source ~/ros2_ws/install/setup.bash')
    if '沒收到 /imu/data' in out:
        return fail('收不到 /imu/data → bringup 沒在跑 → '
                    'ros2 launch stm32_bridge bringup.launch.py')
    m = re.search(r'bias=[-0-9.]+ rad/s = ([+-][0-9.]+) deg/s', out)
    if not m:
        return fail('量不到陀螺零漂：%s' % out.strip()[:120])
    bias = abs(float(m.group(1)))
    if bias >= GYRO_LIMIT_DEG_S:
        return fail('零漂 %.4f °/s ≥ %.1f → 按 STM32 RESET 靜置 3 秒重校'
                    % (bias, GYRO_LIMIT_DEG_S))
    return ok('零漂 %.4f °/s' % bias)


def check_human():
    print('5. 只有人能確認的項目（逐條回答 y）')
    good = True
    for item in HUMAN_CHECKS:
        ans = input('  ? %s\n    確認了嗎 [y/N]: ' % item).strip().lower()
        if ans != 'y':
            good = fail('未確認：%s' % item[:40])
    return good


def main():
    print('=== EX4 / RM3 preflight（任一項不過就不准啟動自主探索）===\n')
    results = [check_injection_evidence(), check_speed_limit(),
               check_no_leftover_nodes(), check_gyro_bias()]
    print()
    if '--no-human' not in sys.argv:
        results.append(check_human())
        print()

    if all(results):
        print('=== 全部通過 ===')
        print('啟動指令（唯一合法入口，不要分開起 bringup/nav2/explorer）：')
        print('    ros2 launch stm32_bridge explore.launch.py')
        return 0
    print('=== 有項目未通過 → 禁止啟動 ===')
    return 1


if __name__ == '__main__':
    sys.exit(main())
