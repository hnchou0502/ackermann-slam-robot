"""自主探索節點：訂 /map 找未探索邊界，經 Nav2 一個一個開過去（EX2，2026-08-11）。

所有判斷邏輯都在零 ROS 的 `explore_math.py`（筆電就能把邊界情境測完）；
這個檔只負責「接 ROS 的線」——訂閱、查 TF、送 action、把狀態機的裁決付諸執行。

**安全設計（不是可選的）**：本節點是 EX3 supervisor 底下的必要子行程。
遇到下列情況一律**立刻結束行程**，由 launch 的 OnProcessExit → Shutdown 把整串關掉，
serial 一斷韌體 500 ms watchdog 就停車：
  - 取消目標後 3 秒內沒等到確認（無法確定車還會不會動）
  - 手上還有目標時 action server 消失（那個目標可能還在跑，而我們已經管不到它）
「不再送新目標、等 Nav2 自己停」不算安全反應——沒有任何機制保證已經在跑的目標會終止。

用法（桌測不動車：把 Nav2 關掉即可，本節點會停在等 server 的狀態）：
    ros2 run stm32_bridge frontier_explorer
實跑一律經 EX3 的 explore.launch.py supervisor 啟動，不要單獨起。
"""
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from stm32_bridge.explore_math import (
    Blacklist, GoalLifecycle, Grid, MapHealth,
    choose_goal, classify_no_goal, extract_frontiers, judge_goal_result,
    ARRIVED_NO_GAIN, FAILED, PREEMPTED, PREEMPTED_NO_GAIN,
    should_preempt, NOT_READY, NO_FRONTIER,
    DEFAULT_BLACKLIST_RADIUS_M, DEFAULT_CANCEL_TIMEOUT_S, DEFAULT_CLEARANCE_M,
    DEFAULT_MAP_STALE_TIMEOUT_S, DEFAULT_MIN_FRONTIER_CELLS,
    DEFAULT_PREEMPT_RATIO, DEFAULT_RETREAT_M,
)

# /map 是 latched：訂閱端 QoS 必須對上，否則車靜止不發新圖時完全收不到
# （2026-08-11 EX1 診斷時用預設 QoS 訂 /map 就是一片空白）。
MAP_QOS = QoSProfile(depth=1)
MAP_QOS.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
MAP_QOS.reliability = QoSReliabilityPolicy.RELIABLE


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        p = self.declare_parameter
        self.map_topic = p('map_topic', '/map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.map_frame = p('map_frame', 'map').value
        self.replan_period = p('replan_period', 15.0).value
        self.min_frontier_cells = p('min_frontier_cells', DEFAULT_MIN_FRONTIER_CELLS).value
        self.retreat = p('retreat_m', DEFAULT_RETREAT_M).value
        self.clearance = p('clearance_m', DEFAULT_CLEARANCE_M).value
        self.preempt_ratio = p('preempt_ratio', DEFAULT_PREEMPT_RATIO).value
        # 0.0 ＝不限時。EX3 會把預設改成 600（10 分鐘）並鎖在參數測試裡。
        self.explore_timeout = p('explore_timeout_s', 0.0).value
        # 送第一個目標前必須等這個 lifecycle 節點進入 active（空字串＝不等）。
        # 為什麼（2026-09-24 EX4 run4 查到，run1/run3 也有）：navigate_to_pose 的 action
        # server 在 configure 階段就存在，server_is_ready() 會回 True，但 bt_navigator
        # 要到 activate 才受理目標——這 2–3 秒內送出的目標全被拒，而被拒＝進黑名單，
        # 於是**車旁邊最近的邊界一開場就被封掉**。
        self.nav2_lifecycle_node = p('nav2_lifecycle_node', 'bt_navigator').value

        self.health = MapHealth(
            timeout=p('map_stale_timeout_s', DEFAULT_MAP_STALE_TIMEOUT_S).value)
        self.goal = GoalLifecycle(
            cancel_timeout=p('cancel_timeout_s', DEFAULT_CANCEL_TIMEOUT_S).value)
        self.blacklist = Blacklist(
            radius=p('blacklist_radius_m', DEFAULT_BLACKLIST_RADIUS_M).value)

        self.grid = None
        self.map_stamp = None
        self.map_is_new = False
        self.current_goal = None            # (x, y, yaw)
        self.current_centroid = None
        self.known_at_send = None           # 送目標當下的已知格數（判斷這趟有沒有收穫）
        self._preempted_gen = None          # 哪一個世代是「我們自己換目標」而取消的
        self._handle = None                 # (generation, goal_handle)；按世代綁定，
                                            # 否則 PENDING 階段取消時可能操作到上一個 handle
        self.last_plan_time = None
        self.started_at = None
        self.done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, MAP_QOS)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_active = not self.nav2_lifecycle_node
        self._nav2_state_pending = False
        self.nav2_state_cli = (self.create_client(
            GetState, '%s/get_state' % self.nav2_lifecycle_node)
            if self.nav2_lifecycle_node else None)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            'frontier_explorer 啟動：重評估每 %.0fs、地圖過期 %.0fs、取消逾時 %.0fs、'
            '總時限 %s' % (self.replan_period, self.health.timeout,
                          self.goal.cancel_timeout,
                          '停用' if self.explore_timeout <= 0
                          else '%.0fs' % self.explore_timeout))

    # ------------------------------------------------------------------ 工具
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _die(self, reason):
        """立刻結束行程，讓 supervisor 的 OnProcessExit 接手把車停下來。

        用 os._exit 而不是 rclpy.shutdown()＋return：這裡的目的就是「快點死」，
        正常關機流程可能因為還在等 action 回應而卡住，那正是我們要避開的情況。
        """
        self.get_logger().fatal('FATAL：%s —— explorer 結束行程，交由 supervisor 停車' % reason)
        os._exit(1)

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _stop_car(self):
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------- 訂閱回呼
    def _on_map(self, msg):
        self.grid = Grid(list(msg.data), msg.info.width, msg.info.height,
                         msg.info.resolution,
                         msg.info.origin.position.x, msg.info.origin.position.y)
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        self.map_stamp = stamp
        state, why = self.health.on_map(stamp, self._now())
        if state == MapHealth.PAUSED and why:
            self.get_logger().warn('地圖狀態 PAUSED：%s' % why)
        self.map_is_new = True

    # ------------------------------------------------------------- 主迴圈
    def _tick(self):
        now = self._now()
        if self.started_at is None:
            self.started_at = now

        fatal = self.goal.tick(now)
        if fatal:
            self._die(fatal)

        if not self.ac.server_is_ready():
            if self.goal.on_server_lost(now):
                self._die(self.goal.fatal_reason)
            return                                   # 手上沒目標，等 server 回來

        if self.done:
            return

        state, why = self.health.tick(now)
        if state == MapHealth.PAUSED:
            if self.goal.has_active_goal and self.goal.state != GoalLifecycle.CANCELLING:
                self.get_logger().warn('地圖過期（%s）→ 取消目標並暫停派送' % why)
                self._cancel_current()
            return

        if 0 < self.explore_timeout <= now - self.started_at:
            self.get_logger().warn('達到探索總時限 %.0fs → 收工' % self.explore_timeout)
            if self.goal.has_active_goal and self.goal.state != GoalLifecycle.CANCELLING:
                self._cancel_current()
            else:
                self._finish('time limit reached')
            return

        if not self.nav2_active:
            self._poll_nav2_state()
            return                                   # Nav2 還沒 active：送了也只會被拒

        if self.grid is None:
            return

        due = (self.last_plan_time is None
               or now - self.last_plan_time >= self.replan_period
               or self.map_is_new)

        if self.goal.may_send_goal():
            if due:
                self._plan_and_send(now)
        elif self.goal.state == GoalLifecycle.ACTIVE and due:
            self._maybe_preempt(now)

    def _poll_nav2_state(self):
        """問 Nav2 的 lifecycle 狀態；拿到 active 才解除「不送目標」。

        查不到（服務還沒出現、回應出錯）一律當作還沒好＝繼續等，由總時限兜底。
        """
        if self._nav2_state_pending or not self.nav2_state_cli.service_is_ready():
            return
        self._nav2_state_pending = True
        self.nav2_state_cli.call_async(GetState.Request()).add_done_callback(
            self._on_nav2_state)

    def _on_nav2_state(self, future):
        self._nav2_state_pending = False
        try:
            state = future.result().current_state
        except Exception:
            return
        if state.id == State.PRIMARY_STATE_ACTIVE and not self.nav2_active:
            self.nav2_active = True
            self.get_logger().info('%s 已 active → 開始派送目標' % self.nav2_lifecycle_node)

    def _frontiers(self):
        return extract_frontiers(self.grid, min_cells=self.min_frontier_cells)

    def _known_cells(self):
        return sum(1 for v in self.grid.data if v >= 0) if self.grid else 0

    def _plan_and_send(self, now):
        self.map_is_new = False
        self.last_plan_time = now
        xy = self._robot_xy()
        if xy is None:
            self.get_logger().warn('查不到 %s→%s 的變換，這輪跳過'
                                   % (self.map_frame, self.base_frame))
            return

        frontiers = self._frontiers()
        goal, frontier = choose_goal(self.grid, frontiers, xy, self.blacklist,
                                     self.retreat, self.clearance)
        if goal is None:
            # ⚠ 不可以一律當成「探索完成」——choose_goal 的 None 混了三種狀況，
            #   把「暫時產不出目標」當成完成，正是 2026-08-30 EX4 三趟全部提早收工的原因
            #   （run3：7 個叢集還在、只有 3 個試過，就宣告完成，覆蓋率 39%）。
            verdict = classify_no_goal(self.grid, frontiers, self.blacklist,
                                       self.retreat, self.clearance)
            if verdict == NOT_READY:
                known = self._known_cells()
                total = len(self.grid.data) if self.grid else 0
                self.get_logger().warn(
                    '還不能動：%d 個邊界叢集都退不出合法目標（已知 %d/%d 格＝%.1f%%）'
                    '——這**不是**探索完成，等地圖長大後重評估'
                    % (len(frontiers), known, total, 100.0 * known / max(1, total)))
                return
            why = ('no frontier left' if verdict == NO_FRONTIER
                   else 'all reachable frontiers blacklisted')
            self._finish('%s (%d clusters, %d blacklisted)'
                         % (why, len(frontiers), len(self.blacklist)))
            return

        self.current_goal = goal
        self.current_centroid = frontier.centroid
        self.known_at_send = self._known_cells()
        self._handle = None                 # 送新目標前先清掉舊 handle（Codex F1）
        gen = self.goal.on_send()
        self.get_logger().info(
            '目標 #%d → (%.2f, %.2f) 朝向 %.0f°（邊界 %d 格，剩 %d 叢）'
            % (gen, goal[0], goal[1], math.degrees(goal[2]), frontier.size,
               len(frontiers)))

        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self.map_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(goal[0])
        msg.pose.pose.position.y = float(goal[1])
        msg.pose.pose.orientation.z = math.sin(goal[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(goal[2] / 2.0)

        fut = self.ac.send_goal_async(msg)
        fut.add_done_callback(lambda f, g=gen: self._on_goal_response(f, g))

    def _on_goal_response(self, future, gen):
        try:
            handle = future.result()
        except Exception as e:                        # server 在送出途中掛掉
            self.get_logger().warn('送目標失敗：%s' % e)
            if self.goal.on_server_lost(self._now()):
                self._die(self.goal.fatal_reason)
            return

        if not handle.accepted:
            if self.goal.on_rejected(gen) and self.current_goal is not None:
                self.get_logger().warn('目標 #%d 被拒 → 加入黑名單' % gen)
                self.blacklist.add(self.current_goal[0], self.current_goal[1])
            return

        if not self.goal.on_accepted(gen):
            return                                    # 已經不是當前世代，丟棄
        self._handle = (gen, handle)
        handle.get_result_async().add_done_callback(
            lambda f, g=gen: self._on_result(f, g))

    def _on_result(self, future, gen):
        """目標的終態回報。**這裡才是解除「不得送新目標」封鎖的唯一入口**（Codex F1）。"""
        try:
            status = future.result().status
        except Exception:
            status = None
        if self._handle is not None and self._handle[0] == gen:
            self._handle = None
        if not self.goal.on_result(gen):
            return                                    # late result 或已收工，丟棄
        if self.current_goal is not None:
            # 判定邏輯在零 ROS 的 explore_math.judge_goal_result（2026-09-24）
            after = self._known_cells()
            listed, kind = judge_goal_result(status, self._preempted_gen == gen,
                                             self.known_at_send, after)
            if kind == FAILED:
                msg = '目標 #%d 未達成（狀態 %s）→ 加入黑名單' % (gen, status)
            elif kind == ARRIVED_NO_GAIN:
                msg = '目標 #%d 抵達但地圖沒有新增（%d 格）→ 加入黑名單，改去別處' % (gen, after)
            elif kind == PREEMPTED_NO_GAIN:
                msg = '目標 #%d 已換掉且途中地圖沒有新增（%d 格）→ 加入黑名單' % (gen, after)
            elif kind == PREEMPTED:
                msg = '目標 #%d 已換掉（途中地圖 %d → %d 格，不列黑名單）' % (
                    gen, self.known_at_send or 0, after)
            else:
                msg = '目標 #%d 抵達，地圖 %d → %d 格' % (gen, self.known_at_send or 0, after)
            (self.get_logger().warn if listed else self.get_logger().info)(msg)
            if listed:
                self.blacklist.add(self.current_goal[0], self.current_goal[1])
        self.current_goal = None
        self.current_centroid = None

    def _maybe_preempt(self, now):
        self.map_is_new = False
        self.last_plan_time = now
        xy = self._robot_xy()
        if xy is None or self.current_goal is None:
            return
        do, why = should_preempt(self.current_goal, self.current_centroid, self.grid,
                                 self._frontiers(), xy, self.blacklist,
                                 self.preempt_ratio)
        if do:
            self.get_logger().info('換目標：%s' % why)
            self._preempted_gen = self.goal.generation
            self._cancel_current()

    def _cancel_current(self):
        """取消現有目標。**取得終態前不會送新目標**；逾時或取消被拒就自殺。

        ⚠ 不可把「cancel 回應到達」當成「舊目標已結束」（Codex F1，2026-08-11）：
          ROS 的 cancel 回應只代表 server 收下請求，goal 可能還在 CANCELING 跑著。
          真正解除封鎖的是同 generation 的 result 終態（見 _on_result）。
        """
        if not self.goal.request_cancel(self._now()):
            return
        gen = self.goal.generation
        if self._handle is None or self._handle[0] != gen:
            # 還沒拿到這個世代的 handle（PENDING 階段就要取消）：沒有東西可取消，
            # 但目標可能已經在 server 端被接受了 → 交給 tick 的逾時判定處理。
            self.get_logger().warn('目標 #%d 尚無 handle 可取消，等待終態或逾時' % gen)
            return
        self._handle[1].cancel_goal_async().add_done_callback(
            lambda f, g=gen: self._on_cancel_response(f, g))

    def _on_cancel_response(self, future, gen):
        """取消請求的回應。被拒或出錯＝我們管不到那個仍在跑的目標 → 致命。"""
        try:
            resp = future.result()
        except Exception as e:
            self.goal._fatal('cancel request failed: %s' % e)
            self._die(self.goal.fatal_reason)
            return
        # goals_canceling 空 ＝ server 沒有取消任何目標（拒絕或目標已不存在）
        if not getattr(resp, 'goals_canceling', None):
            if self.goal.on_cancel_rejected(gen):
                self._die(self.goal.fatal_reason)
            return
        if self.goal.on_cancel_accepted(gen):
            self.get_logger().info('目標 #%d 取消已受理，等待終態回報' % gen)

    def _finish(self, why):
        self.done = True
        self.goal.finish()
        self._stop_car()
        self.get_logger().info('EXPLORATION DONE：%s' % why)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
