"""explore：自主探索的唯一啟動入口＝安全主不變量的實作（EX3，2026-08-11）。

一個 launch 把整串拉起來：bringup（bridge／EKF／光達）＋ Nav2（slam 模式）＋ frontier_explorer。

⚠ **實跑自主探索一律用這個 launch，不要分開起 bringup／nav2／explorer**。
   分開起會繞過下面兩道急停鏈，車失控時沒有任何機制讓它停下來。

安全主不變量
------------
    任何必要元件（explorer／Nav2／bridge／光達驅動）異常退出
      ⇒ 本 launch 全體關閉 ⇒ serial 斷流 ⇒ 韌體 500 ms watchdog 停車回正

兩道機制，都不依賴人的操作：

  機制 a｜任一子行程退出 → 全域 Shutdown
      `OnProcessExit` 不指定 target_action＝監聽**任何**子行程退出。本堆疊裡所有節點
      都是常駐的（沒有跑完就正常結束的一次性行程），所以「有人退出」永遠代表出事了。
      explorer 自己偵測到危險時會 os._exit(1)，走的也是這條。

  機制 b｜親死子亡（pdeathsig）
      每個節點以 `setpriv --pdeathsig=SIGKILL` 當前綴啟動。**連 supervisor 本體被
      SIGKILL（機制 a 完全來不及跑）也不會留下孤兒繼續發 /cmd_vel**——kernel 直接收殺。
      2026-08-11 在 car1 Pi 實測 setpriv (util-linux 2.39.3) 的 pdeathsig 確實生效。
      關掉＝`use_pdeathsig:=false`（只有在 setpriv 不可用的機器上才該關）。

人是第二道，不是第一道：急停順序＝Ctrl-C 本終端 →（無效時）拔 STM32 USB 線。
建議把終端標題設成 EXPLORE-SUPERVISOR，免得急的時候按錯視窗。

用法：
    ros2 launch stm32_bridge explore.launch.py
    ros2 launch stm32_bridge explore.launch.py explore_timeout_s:=300   # 縮短總時限
    ros2 launch stm32_bridge explore.launch.py use_pdeathsig:=false     # 停用機制 b
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 機制 b 的前綴。setpriv 會在 exec 目標程式前設好 parent-death-signal，
# 之後不論父行程怎麼死（含 SIGKILL），kernel 都會對子行程送 SIGKILL。
PDEATHSIG_PREFIX = 'setpriv --pdeathsig=SIGKILL'
EXPLORE_REVERSAL_DWELL_S = '0.3'
EXPLORE_MAX_ACCEL = '0.5'


def generate_launch_description():
    pkg = get_package_share_directory('stm32_bridge')
    launch_dir = os.path.join(pkg, 'launch')
    explore_params = os.path.join(pkg, 'config', 'nav2_params_explore.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    use_pdeathsig = LaunchConfiguration('use_pdeathsig')
    explore_timeout = LaunchConfiguration('explore_timeout_s')
    lidar_type = LaunchConfiguration('lidar_type')

    # use_pdeathsig:=false → 空前綴（行為等同沒有機制 b）
    node_prefix = PythonExpression(
        ["'", PDEATHSIG_PREFIX, "' if '", use_pdeathsig, "' == 'true' else ''"])

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup.launch.py')),
        launch_arguments={
            'lidar_type': lidar_type,
            'node_prefix': node_prefix,
            # EX4 限速（2026-09-24）：run5 實際輪速 0.474、送到韌體的只有 0.290，
            # 超標 58% 在換向後 2 s 內＝韌體 PID 沒歸零就反向。見 protocol.CmdShaper。
            #   0.3 s＝EX3 實測從行進到 FBK 歸零 0.20–0.25 s，取其上方
            #   0.5 m/s²＝0→0.25 m/s 用 0.5 s（bridge 20 Hz，每步目標變化 ≤0.025 m/s）
            'cmd_reversal_dwell_s': EXPLORE_REVERSAL_DWELL_S,
            'cmd_max_accel': EXPLORE_MAX_ACCEL,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2.launch.py')),
        launch_arguments={
            'slam': 'true',
            # 兩份參數檔依序疊：後者只覆蓋 desired_linear_vel（見 nav2_params_explore.yaml）
            'params_file': nav2_params,
            # 探索限速覆蓋層（只改 desired_linear_vel，見 nav2_params_explore.yaml）
            'extra_params_file': explore_params,
            'node_prefix': node_prefix,
        }.items(),
    )

    explorer = Node(
        prefix=node_prefix,
        package='stm32_bridge', executable='frontier_explorer',
        name='frontier_explorer', output='screen',
        parameters=[{'explore_timeout_s': explore_timeout}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_pdeathsig', default_value='true'),
        DeclareLaunchArgument('explore_timeout_s', default_value='600.0'),
        DeclareLaunchArgument('lidar_type', default_value='c1'),

        LogInfo(msg='[EXPLORE] 自主探索啟動。急停：先 Ctrl-C 本終端，無效時拔 STM32 USB 線。'),
        bringup,
        nav2,
        explorer,

        # 機制 a：任何子行程退出 → 全體關閉（不指定 target_action ＝監聽全部）
        RegisterEventHandler(OnProcessExit(
            on_exit=[
                LogInfo(msg='[EXPLORE] 偵測到必要元件退出 → 關閉整串，'
                            'serial 斷流後韌體 watchdog 會在 500 ms 內停車。'),
                EmitEvent(event=Shutdown(reason='required component exited')),
            ],
        )),
    ])
