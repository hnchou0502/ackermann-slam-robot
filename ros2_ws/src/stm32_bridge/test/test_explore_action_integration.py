"""frontier_explorer 與真實 Nav2 action 介面的整合測試（EX2，2026-08-11）。

**必須在 Pi（有 ROS 2 Jazzy）上跑**；Windows 無 ROS 會整檔 skip——
純邏輯的部分（狀態機轉移、late result、取消逾時判定）已由 test_explore_math.py
在筆電上測完，這裡只測「接上真的 action server 之後行為對不對」，尤其是：

  **取消目標得不到確認時，explorer 必須真的把自己的行程結束掉**（exit code != 0）。
  這條沒辦法在同一個行程裡用單元測試驗證——因為要驗的就是「行程死掉」這件事本身，
  所以用 subprocess 起真的節點，看它是不是真的死了。

⚠⚠ **必須分兩批跑，不要一次跑完 9 個**（2026-08-11 實測定案）：

    cd ~/ros2_ws && source install/setup.bash && cd src/stm32_bridge
    python3 -m pytest test/test_explore_action_integration.py -q -k "one_goal_at_a_time or rejected_goal_is_blacklisted or response_never_arrives or waits_when_no_action or exploration_done"   # 批次 1＝5 個
    python3 -m pytest test/test_explore_action_integration.py -q         -k "cancel_rejected or delayed_termination or server_disappearing or shutdown_stops"
                                                                   # 批次 2＝4 個
    python3 -m pytest test/test_explore_action_integration.py -q -k no_goal_until_nav2_active   # 批次 3＝1 個（2026-09-24）

⚠ 改測試名時**務必同步改上面的 -k**（Codex R3-F2，2026-08-11 踩過：改名後篩選字串沒更新，
   照文件只跑得到 7/9，漏掉的正是當輪新增的兩個關鍵測試）。改完用
   `--collect-only -q | tail -1` 確認兩批分別收集到 5 與 4 個。

原因：一次跑滿 9 個時，第 6 個起會拋
`RuntimeError: Context.init() must only be called once`——rclpy 的**全域預設 context
不能反覆 init/shutdown**。分兩批（各自是獨立的 python 行程）就沒有這個問題，
9 個測試全部驗證得到。

**試過但放棄的解法**（別再走一次）：
  1. autouse fixture 清理殘留 explorer 行程 → 無效（根因不是行程殘留）
  2. 每個測試換一個 ROS domain → 無效，且每測試從 41 秒變 137 秒（新 domain 的
     DDS discovery 要從零建立）
  3. 每個測試用獨立 `rclpy.context.Context()` → **確實修掉了那個例外**，但更慢：
     4 個測試就用掉 850 秒，整批跑不完。修對了根因卻換來跑不完，不划算。

⚠ 教訓：前兩次嘗試都是在**猜**根因，因為一直用 `-q | tail` 把錯誤訊息截掉了。
   拿到那行 `Context.init() must only be called once` 只花 9 分鐘，瞎猜浪費了半小時。
"""
import os
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy', reason='需要 ROS 2 環境（在 Pi 上跑）')

from action_msgs.msg import GoalStatus                              # noqa: E402
from geometry_msgs.msg import TransformStamped                      # noqa: E402
from lifecycle_msgs.msg import State                                # noqa: E402
from lifecycle_msgs.srv import GetState                             # noqa: E402
from nav2_msgs.action import NavigateToPose                         # noqa: E402
from nav_msgs.msg import OccupancyGrid                              # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.node import Node                                         # noqa: E402
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,             # noqa: E402
                       QoSReliabilityPolicy)
from tf2_ros import StaticTransformBroadcaster                      # noqa: E402

MAP_QOS = QoSProfile(depth=1)
MAP_QOS.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
MAP_QOS.reliability = QoSReliabilityPolicy.RELIABLE

# 測試用的 ROS domain（避開車上正在跑的 domain 0）。
#
# ⚠ **不要改成「每個測試換一個 domain」**：2026-08-11 試過，每個測試從 41 秒變成
#   137 秒（新 domain 的 DDS discovery 要從零建立），整批連跑不完。
#   當初加它是因為誤判整批失敗的原因是「discovery 殘影累積」——真正的原因是
#   `rclpy.init()` 重複呼叫（見 ros fixture 的說明），跟 domain 無關。
DOMAIN = '77'


class Fixture(Node):
    """假的 Nav2：發地圖、發 TF、開一個可調行為的 navigate_to_pose server。"""

    def __init__(self, accept=True, ack_cancel=True, auto_result=None,
                 full_known=False, cancel_delay=0.0, context=None,
                 nav_state=State.PRIMARY_STATE_ACTIVE):
        super().__init__('ex2_fixture')
        self.accept = accept
        self.ack_cancel = ack_cancel
        self.auto_result = auto_result          # None＝不主動回結果（讓目標一直「執行中」）
        self.full_known = full_known            # True＝發一張沒有未知區的地圖
        self.cancel_delay = cancel_delay        # 受理取消後拖這麼久才進終態
                                                # （模擬「受理≠已結束」，Codex F1）
        self.nav_state = nav_state              # 假 bt_navigator 的 lifecycle 狀態（可中途改）
        self.goals_received = 0
        self.active_goals = 0
        self.max_concurrent = 0
        self.cancel_requests = 0

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self.tf_bc = StaticTransformBroadcaster(self)
        self.srv = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._execute)
        # explorer 送第一個目標前會問 bt_navigator 是否 active（2026-09-24 EX4 run4）
        self.state_srv = self.create_service(
            GetState, 'bt_navigator/get_state', self._on_get_state)
        self.create_timer(0.5, self.publish_map)
        self._publish_tf()
        # executor 必須綁在同一個 context 上：rclpy.spin_once() 用的是全域 executor
        # （預設 context），跟自訂 context 的節點不匹配，會在 GuardCondition 炸掉。


    def _publish_tf(self):
        for parent, child in (('map', 'odom'), ('odom', 'base_link')):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            self.tf_bc.sendTransform(t)

    def publish_map(self):
        """左右封牆、上方通往未知區的地圖（full_known=True 則四周全封、無未知區）。

        ⚠ 不要在測試裡用 `fx.publish_map = 別的函式` 換地圖：timer 在建構時就抓住了
        原本的 bound method，換掉屬性完全沒有效果（2026-08-11 踩過，測試假失敗）。
        """
        w, h, res = 21, 16, 0.1
        data = []
        for j in range(h):
            for i in range(w):
                if j >= h - 1:
                    data.append(100 if self.full_known else -1)
                elif i == 0 or i == w - 1 or j == 0:
                    data.append(100)                # 左右與底部是牆
                else:
                    data.append(0)
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.width, m.info.height, m.info.resolution = w, h, res
        m.info.origin.position.x = -1.0
        m.info.origin.position.y = -0.2
        m.info.origin.orientation.w = 1.0
        m.data = data
        self.map_pub.publish(m)

    def _on_get_state(self, req, resp):
        resp.current_state.id = self.nav_state
        return resp

    def _on_goal(self, req):
        self.goals_received += 1
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def _on_cancel(self, goal_handle):
        self.cancel_requests += 1
        return CancelResponse.ACCEPT if self.ack_cancel else CancelResponse.REJECT

    def _execute(self, goal_handle):
        self.active_goals += 1
        self.max_concurrent = max(self.max_concurrent, self.active_goals)
        try:
            deadline = time.time() + (self.auto_result if self.auto_result else 30.0)
            cancel_seen_at = None
            while time.time() < deadline:
                if goal_handle.is_cancel_requested and self.ack_cancel:
                    if cancel_seen_at is None:
                        cancel_seen_at = time.time()
                    # 受理取消後刻意拖延才進終態：驗證 explorer 在這段期間
                    # 絕不送新目標（Codex F1）
                    if time.time() - cancel_seen_at >= self.cancel_delay:
                        goal_handle.canceled()
                        return NavigateToPose.Result()
                time.sleep(0.05)
            if self.auto_result:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return NavigateToPose.Result()
        finally:
            self.active_goals -= 1


def _spin(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def _start_explorer(extra_params=()):
    # 繼承當前測試的 domain（由 ros fixture 設定），不要寫死
    env = dict(os.environ)
    cmd = ['ros2', 'run', 'stm32_bridge', 'frontier_explorer']
    if extra_params:
        cmd += ['--ros-args']
        for kv in extra_params:
            cmd += ['-p', kv]
    return subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)


@pytest.fixture(autouse=True)
def _no_leftover_explorers():
    """每個測試前後都清掉殘留的 explorer。

    ⚠ 沒有這道清理時測試會**互相干擾**（2026-08-11 實測）：前一個測試 terminate 後
    explorer 未必立刻死透，殘留的那個會在同一個 ROS domain 上繼續對 fake server 送目標，
    害下一個測試的 goals_received 斷言失敗——單獨跑該測試卻是綠的，典型的假紅燈。
    只清 explorer（測試專用 domain，見 DOMAIN），不動 bridge/Nav2。
    """
    kill = "pkill -f 'stm32_bridge/frontier_explorer'"
    subprocess.run(kill, shell=True, capture_output=True)
    time.sleep(1.0)
    yield
    subprocess.run(kill, shell=True, capture_output=True)
    time.sleep(1.0)


@pytest.fixture
def ros():
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    rclpy.init()
    yield None
    if rclpy.ok():
        rclpy.shutdown()


def test_explorer_sends_exactly_one_goal_at_a_time(ros):
    fx = Fixture(auto_result=1.0)
    proc = _start_explorer()
    try:
        _spin(fx, 12.0)
        assert fx.goals_received >= 1, 'explorer 應該至少送出一個目標'
        assert fx.max_concurrent <= 1, \
            '任何時刻都不得有兩個目標同時有效（會變成兩個東西同時控制同一台車）'
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        fx.destroy_node()


def test_rejected_goal_is_blacklisted_and_explorer_keeps_running(ros):
    """目標被拒不該讓 explorer 死掉，應該把該點列黑名單改挑別的。"""
    fx = Fixture(accept=False)
    proc = _start_explorer()
    try:
        _spin(fx, 10.0)
        assert fx.goals_received >= 1
        assert proc.poll() is None, '目標被拒不是致命錯誤，explorer 應該還活著'
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        fx.destroy_node()


def test_no_goal_until_nav2_active(ros):
    """Nav2 還在 inactive 時一個目標都不准送；轉 active 後才開始送。

    2026-09-24 EX4 run4（回頭查 run1/run3 也一樣）：action server 在 configure 就存在，
    explorer 早 2–3 秒送目標 → 全被拒 → 最近的邊界一開場就進黑名單。
    """
    fx = Fixture(nav_state=State.PRIMARY_STATE_INACTIVE)
    proc = _start_explorer()
    try:
        _spin(fx, 8.0)
        assert fx.goals_received == 0, 'Nav2 還沒 active 就送了目標（會被拒並進黑名單）'
        assert proc.poll() is None, '等 Nav2 不是錯誤，explorer 應該還活著'
        fx.nav_state = State.PRIMARY_STATE_ACTIVE
        _spin(fx, 8.0)
        assert fx.goals_received >= 1, 'Nav2 active 之後應該開始送目標'
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        fx.destroy_node()


def test_cancel_response_never_arrives_kills_the_process(ros):
    """**取消請求完全沒有回應**（server 卡死）→ explorer 逾時自殺。

    ⚠ 這與 `test_cancel_rejected_kills_the_process` 是**不同路徑**（Codex R2-F3）：
      那個是 server 明確回「拒絕」，這個是連回應都沒有。舊版兩個測試都用
      `ack_cancel=False`，fixture 立刻回 REJECT，等於測了同一條路兩次。

    做法：等 explorer 送出目標後**完全停止 spin fixture**——server 不再處理任何請求。
    explorer 因收不到新地圖而在 map_stale_timeout(10s) 後判定地圖過期 → 送取消 →
    取消請求無人回應 → cancel_timeout 後 fatal。順便涵蓋卡上的 stale-map cancel 路徑。
    """
    fx = Fixture(auto_result=None)
    proc = _start_explorer(extra_params=['cancel_timeout_s:=3.0'])
    try:
        _spin(fx, 14.0)
        assert fx.goals_received >= 1, '要先讓 explorer 送出並被接受一個目標'
        # 之後完全不 spin：server 卡死，地圖也不再發布
        deadline = time.time() + 45.0
        while time.time() < deadline and proc.poll() is None:
            time.sleep(0.5)
        assert proc.poll() is not None,             '取消請求得不到任何回應時，explorer 必須自殺讓 supervisor 停車'
        assert proc.returncode != 0
        out = proc.stdout.read()
        assert 'FATAL' in out
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=10)
        fx.destroy_node()


def test_explorer_waits_when_no_action_server_present(ros):
    """Nav2 還沒起來時，explorer 應該安靜等待，而不是崩掉或亂送目標。"""
    proc = _start_explorer()
    try:
        time.sleep(8.0)
        assert proc.poll() is None, '沒有 action server 時應該等待（手上沒目標，不算致命）'
    finally:
        proc.terminate()
        proc.wait(timeout=10)


def test_exploration_done_when_map_fully_known(ros):
    """地圖沒有未知區時，explorer 應該收工並印 EXPLORATION DONE，不再送目標。"""
    fx = Fixture(auto_result=1.0, full_known=True)
    proc = _start_explorer()
    try:
        _spin(fx, 12.0)
        assert fx.goals_received == 0, '沒有未探索區就不該送任何目標'
        assert proc.poll() is None, '收工不是錯誤，explorer 應該還活著'
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        out = proc.stdout.read()
        fx.destroy_node()
    assert 'EXPLORATION DONE' in out, '沒有邊界可去時必須明確收工，不能靜靜地什麼都不做'


# --- Codex F6（2026-08-11）：卡上逐名要求、先前沒涵蓋的失敗路徑 ---

def test_cancel_rejected_kills_the_process(ros):
    """取消被 server 拒絕 → explorer 必須自殺（我們已無法停止那個仍在跑的目標）。

    先前只測「取消得不到回應」，沒測「回應說拒絕」——後者舊版會誤判成取消成功
    並解除封鎖，正是 F1 的核心。
    """
    fx = Fixture(ack_cancel=False)
    proc = _start_explorer(extra_params=['cancel_timeout_s:=3.0', 'replan_period:=2.0'])
    try:
        deadline = time.time() + 45.0
        while time.time() < deadline and proc.poll() is None:
            _spin(fx, 0.5)
        assert proc.poll() is not None, '取消被拒時 explorer 必須結束行程'
        assert proc.returncode != 0
        assert fx.max_concurrent <= 1, '全程不得有兩個目標同時有效'
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=10)
        fx.destroy_node()


def test_cancel_accepted_but_delayed_termination_never_double_sends(ros):
    """取消被受理但目標拖 5 秒才終止：這段期間 explorer 絕不能送出第二個目標。"""
    fx = Fixture(auto_result=None, cancel_delay=5.0)
    proc = _start_explorer(extra_params=['cancel_timeout_s:=30.0', 'replan_period:=2.0'])
    try:
        _spin(fx, 25.0)
        assert fx.goals_received >= 1
        assert fx.max_concurrent <= 1,             '「受理取消」不等於「目標已結束」，在終態到達前送新目標＝雙重控制'
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        fx.destroy_node()


def test_server_disappearing_with_goal_in_flight_kills_the_process(ros):
    """手上還有目標時 action server 消失 → 那個目標可能還在跑而我們管不到 → 自殺。"""
    fx = Fixture(auto_result=None)
    proc = _start_explorer()
    try:
        _spin(fx, 12.0)
        assert fx.goals_received >= 1, '要先讓 explorer 送出一個目標'
        fx.srv.destroy()                      # server 在 active goal 期間消失
        deadline = time.time() + 30.0
        while time.time() < deadline and proc.poll() is None:
            _spin(fx, 0.5)
        assert proc.poll() is not None, 'active goal 期間 server 消失必須致命'
        assert proc.returncode != 0
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=10)
        fx.destroy_node()


def test_shutdown_stops_sending_goals_and_never_double_sends(ros):
    """explorer 被 SIGTERM 關閉後，不再送出任何新目標，且全程至多一個有效目標。

    ⚠ **本測試不宣稱「關閉時會主動取消目標」**（Codex R2-F3 指出舊版斷言與宣稱不符）：
      explorer 收到 SIGTERM 時 rclpy context 已在關閉流程中，送不出 action 取消請求。
      真實部署裡這不成問題——explorer 一退出就觸發 supervisor 的
      OnProcessExit → 整串關閉（含 Nav2），那個目標連同 server 一起消失。
      這裡的 fake server 不會被關掉，所以目標會留到它自己 abort，屬測試環境差異。
      斷言改為「關閉後不再有新目標」＋「歷史最大併發 <=1」，這才是本測試真正驗到的。
    """
    fx = Fixture(auto_result=None)
    proc = _start_explorer()
    try:
        _spin(fx, 12.0)
        assert fx.goals_received >= 1
        before = fx.goals_received
        proc.terminate()
        proc.wait(timeout=15)
        _spin(fx, 6.0)
        assert fx.goals_received == before, '關閉後不得再送出新目標'
        assert fx.max_concurrent <= 1
    finally:
        if proc.poll() is None:
            proc.kill()
        fx.destroy_node()
