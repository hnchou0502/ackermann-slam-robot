"""EX3 supervisor 與安全參數的結構鎖（2026-08-11；零 ROS，Windows 直接 pytest）。

這些值不是「設定」，是安全設計的一部分：
  - 限速 0.25 m/s：探索時車自己在跑、人只在旁邊看，必須比遙控時（0.35）更保守
  - 探索總時限 600 s：車失去理智時的最後一道時間閘
  - pdeathsig 前綴：supervisor 本體被 SIGKILL 時，保證沒有孤兒節點繼續發 /cmd_vel
  - OnProcessExit → Shutdown：任一必要元件退出就關掉整串，讓韌體 watchdog 停車

改動任何一項都要連同這個檔一起改，並在 CLAUDE.md 記下實測理由。
"""
import ast
import os
import math
import re

import yaml

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, '..', 'launch', 'explore.launch.py')
NAV2_LAUNCH = os.path.join(HERE, '..', 'launch', 'nav2.launch.py')
BRINGUP_LAUNCH = os.path.join(HERE, '..', 'launch', 'bringup.launch.py')
EXPLORE_PARAMS = os.path.join(HERE, '..', 'config', 'nav2_params_explore.yaml')
NAV2_PARAMS = os.path.join(HERE, '..', 'config', 'nav2_params.yaml')
NONE_PARAMS = os.path.join(HERE, '..', 'config', 'params_override_none.yaml')


def _read(p):
    return open(p, encoding='utf-8').read()


def _launch_defaults(path):
    out = {}
    for call in ast.walk(ast.parse(_read(path))):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == 'DeclareLaunchArgument' and call.args):
            key = call.args[0].value if isinstance(call.args[0], ast.Constant) else None
            val = None
            for k in call.keywords:
                if k.arg == 'default_value' and isinstance(k.value, ast.Constant):
                    val = k.value.value
            out[key] = val
    return out


# ------------------------------------------------------------------ 限速

def test_explore_speed_limit_is_locked_at_025():
    cfg = yaml.safe_load(_read(EXPLORE_PARAMS))
    v = cfg['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']
    assert v == 0.25, (
        '探索限速必須是 0.25 m/s（EX4 以 FBK 輪速驗收 <=0.28）。'
        '實際 %r——改這個值等於改安全規格' % v)


def test_explore_override_is_a_thin_layer_not_a_copy():
    """覆蓋層只能寫要改的那幾項；整份複製 nav2_params.yaml 會變成兩份各自演化的大檔。

    做法＝**逐項白名單**，而不是只限制頂層節點數。新增覆蓋項時必須同步改這裡，
    強迫每一項都留下「為什麼探索時要跟一般導航不一樣」的理由。

    目前允許三項：
      1. 限速（探索時車自己在跑，要比遙控保守）
      2/3. 全域與局部代價地圖的膨脹半徑——主檔的 0.25 小於車的外接半徑 0.318，
           2026-08-30 EX4 首跑因此規劃失敗 28 次、警告 142 次；主檔的值是為 4 m
           小房間折衷的，不能直接改大（會連累 EX1），故只在探索時覆蓋。
    """
    cfg = yaml.safe_load(_read(EXPLORE_PARAMS))
    assert sorted(cfg.keys()) == ['controller_server', 'global_costmap', 'local_costmap'], \
        '覆蓋層碰了白名單以外的節點，實際 %s' % sorted(cfg)

    fp = cfg['controller_server']['ros__parameters']['FollowPath']
    assert list(fp.keys()) == ['desired_linear_vel'], \
        'controller_server 目前只該覆蓋限速一項，實際 %s' % list(fp)

    for name in ('global_costmap', 'local_costmap'):
        params = cfg[name][name]['ros__parameters']
        assert list(params.keys()) == ['inflation_layer'], \
            '%s 目前只該覆蓋 inflation_layer，實際 %s' % (name, list(params))
        assert list(params['inflation_layer'].keys()) == ['inflation_radius'], \
            '%s 只該覆蓋 inflation_radius，實際 %s' % (name, list(params['inflation_layer']))


def test_explore_inflation_covers_the_robot_circumscribed_radius():
    """膨脹半徑必須 ≥ 車的外接半徑，否則 Nav2 的碰撞檢查會退化、規劃大量失敗。

    外接半徑由 footprint 算出：sqrt(0.28² + 0.12²) = 0.305（Nav2 實際報 0.3178，含邊距）。
    2026-08-30 EX4 首跑用主檔的 0.25 → 警告 142 次、no valid path found 28 次、
    8 個目標全進黑名單、覆蓋率只有 46.7%。
    """
    cfg = yaml.safe_load(_read(EXPLORE_PARAMS))
    circumscribed = math.hypot(0.28, 0.12)
    for name in ('global_costmap', 'local_costmap'):
        r = cfg[name][name]['ros__parameters']['inflation_layer']['inflation_radius']
        assert r >= circumscribed, \
            '%s 膨脹半徑 %.3f 小於車的外接半徑 %.3f——Nav2 會一路警告並規劃失敗' \
            % (name, r, circumscribed)


def test_base_params_still_has_the_faster_normal_speed():
    """基礎檔維持一般導航的 0.35——覆蓋層要有東西可覆蓋才有意義。"""
    m = re.search(r'desired_linear_vel:\s*([0-9.]+)', _read(NAV2_PARAMS))
    assert m and float(m.group(1)) == 0.35, \
        '一般導航速度應維持 0.35；若這裡也改成 0.25，覆蓋層就變成沒作用的裝飾'


def test_empty_override_is_valid_and_changes_nothing():
    cfg = yaml.safe_load(_read(NONE_PARAMS))
    assert cfg == {'/**': {'ros__parameters': {}}}, \
        '預設覆蓋層必須是合法但什麼都不改的 yaml（parameters 列表不接受空字串）'


# ------------------------------------------------------------------ supervisor

def test_supervisor_defaults():
    d = _launch_defaults(LAUNCH)
    assert d.get('explore_timeout_s') == '600.0', \
        '探索總時限預設 10 分鐘，實際 %r' % d.get('explore_timeout_s')
    assert d.get('use_pdeathsig') == 'true', \
        'pdeathsig 預設必須開啟——關掉就等於允許 supervisor 被強殺後留下孤兒節點'
    assert d.get('lidar_type') == 'c1', 'car1 現役光達'


def test_pdeathsig_prefix_is_a_real_kill_signal():
    src = _read(LAUNCH)
    m = re.search(r"PDEATHSIG_PREFIX\s*=\s*'([^']+)'", src)
    assert m, '找不到 PDEATHSIG_PREFIX'
    assert m.group(1) == 'setpriv --pdeathsig=SIGKILL', \
        ('前綴必須用 SIGKILL：SIGTERM 可以被節點忽略或處理失敗，'
         '而這條路徑的前提就是「父行程已經死到來不及做任何事」。實際 %r' % m.group(1))


def test_supervisor_shuts_everything_down_when_any_process_exits():
    src = _read(LAUNCH)
    assert 'OnProcessExit' in src and 'Shutdown' in src, \
        '機制 a 不見了：任一必要元件退出必須關閉整串'
    # 不得指定 target_action——那會變成只監聽某一個節點
    m = re.search(r'OnProcessExit\((.*?)\)\)', src, re.S)
    assert m and 'target_action' not in m.group(1), \
        'OnProcessExit 不可綁定單一節點，必須監聽所有子行程'


def test_supervisor_starts_the_whole_stack():
    """分開起會繞過急停鏈，所以 supervisor 必須自己把三塊都拉起來。"""
    src = _read(LAUNCH)
    assert 'bringup.launch.py' in src, '缺 bringup（bridge/EKF/光達）'
    assert 'nav2.launch.py' in src, '缺 Nav2'
    assert 'frontier_explorer' in src, '缺 explorer'
    assert "'slam': 'true'" in src, 'Nav2 必須用 slam 模式（探索時沒有預載地圖）'


def test_explore_params_are_actually_passed_to_nav2():
    """限速覆蓋層有寫進 yaml 但沒傳給 Nav2 的話，車還是會用 0.35 跑。"""
    src = _read(LAUNCH)
    assert 'extra_params_file' in src and 'explore_params' in src, \
        '限速覆蓋層必須經 extra_params_file 傳進 nav2.launch.py'


def test_nav2_launch_applies_override_after_base_params():
    """ROS2 的 parameters 列表是後者覆蓋前者——順序反了限速就不會生效。"""
    src = _read(NAV2_LAUNCH)
    m = re.search(r'common\s*=\s*\[([^\]]+)\]', src)
    assert m, '找不到 common 參數列表'
    order = m.group(1)
    assert order.index('params_file') < order.index('extra_params_file'), \
        '覆蓋層必須排在基礎參數之後，否則會被基礎值蓋回去'


# ------------------------------------------------------------------ 不破壞現有跑法

def test_node_prefix_defaults_to_empty_everywhere():
    """node_prefix 預設空字串＝手動起 bringup／nav2 的舊跑法行為完全不變。"""
    for path in (NAV2_LAUNCH, BRINGUP_LAUNCH):
        d = _launch_defaults(path)
        assert d.get('node_prefix') == '', \
            '%s 的 node_prefix 預設必須是空字串，實際 %r' % (
                os.path.basename(path), d.get('node_prefix'))


def test_every_node_gets_the_prefix():
    """漏掉任何一個節點，那個節點就會在 supervisor 被強殺後變成孤兒。"""
    for path in (NAV2_LAUNCH, BRINGUP_LAUNCH):
        src = _read(path)
        n_nodes = len(re.findall(r'\b(?:Lifecycle)?Node\(\n', src))
        n_prefix = src.count('prefix=node_prefix')
        assert n_nodes == n_prefix, \
            '%s：%d 個節點但只有 %d 個帶 prefix' % (
                os.path.basename(path), n_nodes, n_prefix)


# --- EX4 限速：bridge 指令整形（2026-09-24）---------------------------------

def test_bringup_cmd_shaping_is_off_by_default():
    """一般 bringup（遙控、EX1、RM 線）不得被整形影響：預設 0＝直通。"""
    d = _launch_defaults(BRINGUP_LAUNCH)
    assert d['cmd_reversal_dwell_s'] == '0.0'
    assert d['cmd_max_accel'] == '0.0'


def test_explore_turns_cmd_shaping_on():
    """探索 supervisor 必須把整形打開並傳給 bringup（值的依據見 explore.launch.py 註解）。"""
    src = _read(LAUNCH)
    assert re.search(r"EXPLORE_REVERSAL_DWELL_S\s*=\s*'0\.3'", src)
    assert re.search(r"EXPLORE_MAX_ACCEL\s*=\s*'0\.5'", src)
    assert "'cmd_reversal_dwell_s': EXPLORE_REVERSAL_DWELL_S" in src
    assert "'cmd_max_accel': EXPLORE_MAX_ACCEL" in src


def test_bringup_passes_shaping_args_to_bridge():
    src = _read(BRINGUP_LAUNCH)
    assert "'reversal_dwell_s': ParameterValue(cmd_reversal_dwell_s, value_type=float)" in src
    assert "'max_linear_accel': ParameterValue(cmd_max_accel, value_type=float)" in src
