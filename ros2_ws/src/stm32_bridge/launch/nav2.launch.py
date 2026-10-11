"""nav2：單車自主導航堆疊。兩種定位來源二選一：

  slam:=false（預設）＝**用既有靜態地圖**：map_server 載入 yaml + AMCL 定位。
  slam:=true         ＝**邊建圖邊導航**（EX1，2026-08-11）：slam_toolbox 一邊建圖
                       一邊發 map->odom，取代 map_server + AMCL。不需要預載地圖。

建立 2026-07-12。起手包＝docs/design/nav2_starter_pack.md。

⚠ 這個 launch 只起「導航堆疊」，不含 bridge/EKF/光達。實跑要**兩個 launch 一起**：
  終端1(車體+感測+TF)：ros2 launch stm32_bridge bringup.launch.py
  終端2(導航堆疊)     ：ros2 launch stm32_bridge nav2.launch.py            # 有現成地圖
       或             ：ros2 launch stm32_bridge nav2.launch.py slam:=true  # 沒地圖，邊建邊導
  終端3(給目標,選)   ：RViz2 的 "2D Pose Estimate" 設初始位姿 -> "Nav2 Goal" 給目標
  (或 rviz 用 nav2 預設 config：ros2 launch nav2_bringup rviz_launch.py)
  ⚠ slam:=true 時不要點 "2D Pose Estimate"（那是給 AMCL 的，此模式沒有 AMCL）；
    車的位置由 slam_toolbox 自己決定，起點就是地圖原點。

TF 前提：bringup 已發 odom->base_link->laser_frame；本堆疊補上 map->odom 封頂 TF 樹
        （slam:=false 由 AMCL 發、slam:=true 由 slam_toolbox 發——兩者只能有一個在跑，
          同時起會兩個節點搶發 map->odom，TF 樹打架且不報錯，故本 launch 以參數互斥）。

選項：
  slam:=true           邊建圖邊導航(不載入靜態地圖，map/init_x/init_y/init_yaw 全部忽略)
  map:=<path>          靜態地圖 yaml(預設 ~/maps/map1.yaml，Pi 上的建圖產物；slam 模式不用)
  params_file:=<path>  nav2 參數(預設本套件 config/nav2_params.yaml)
  slam_params_file:=<path>  slam_toolbox 參數(預設本套件 config/slam_toolbox_mapping.yaml)
  autostart:=false     不自動 configure+activate 生命週期節點(除錯用，預設 true)
  use_sim_time:=true   回放 bag/模擬時鐘(實車預設 false)

⚠ slam 模式下 planner 的 allow_unknown 仍是 false（見 nav2_params.yaml）：目標點與整條路徑
  都必須落在已掃過的區域，點到還沒掃到的灰色區會直接規劃失敗。這是刻意的——讓車開進沒掃過
  的地方＝拿光達當保險桿。EX2 的自主探索因此把目標點從邊界往已知區退 0.5m。

⚠ 單車版：所有 lifecycle 節點在根命名空間。雙車化需另做(見 nav2_params.yaml 檔頭說明 + bringup 的 TF remap)。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('stm32_bridge')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    default_slam_params = os.path.join(pkg, 'config', 'slam_toolbox_mapping.yaml')
    default_extra_params = os.path.join(pkg, 'config', 'params_override_none.yaml')
    default_map = os.path.join(os.path.expanduser('~'), 'maps', 'map1.yaml')

    # EX3 安全機制 b「親死子亡」：supervisor 以 'setpriv --pdeathsig=SIGKILL' 當前綴
    # 啟動每個節點，這樣連 supervisor 本身被 SIGKILL，子行程也會被 kernel 一起收掉，
    # 不會留下孤兒繼續發 /cmd_vel。預設空字串＝原本的跑法完全不受影響。
    node_prefix = LaunchConfiguration('node_prefix')

    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    # 疊在 params_file 之上的覆蓋層（後者勝出）。探索模式用它把 RPP 限速壓到 0.25 m/s；
    # 預設是一份什麼都不改的空 yaml，因為 parameters 列表不接受空字串。
    extra_params_file = LaunchConfiguration('extra_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # AMCL 初始位置（見 nav2_params.yaml 的啟動競態註解）。預設＝地圖原點，
    # 即「車停在當初開始建圖的地方」。停別處就量一次 map->base_link 帶進來。
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')

    # 生命週期管理器要拉起的節點。導航這幾個兩種模式共用。
    nav_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    # slam:=false 時多管定位這兩個（順序＝先地圖再 AMCL，再接導航）。
    localization_lifecycle_nodes = ['map_server', 'amcl'] + nav_lifecycle_nodes

    common = [params_file, extra_params_file, {'use_sim_time': use_sim_time}]

    # --- slam:=true 的建圖節點 ---
    # ⚠ 不把 slam_toolbox 交給 nav2 的 lifecycle_manager 管：那條路要節點建立 bond
    #   連線（slam_toolbox 的 use_lifecycle_manager 參數），沒 bond 會讓 manager 逾時
    #   而整批啟動失敗。改用 launch 原生的 EmitEvent 驅動 configure -> activate，
    #   與 slam_toolbox 官方 online_async_launch.py 同一套做法。
    slam_toolbox_node = LifecycleNode(
        prefix=node_prefix,
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[slam_params_file, {
            'use_sim_time': use_sim_time,
            'use_lifecycle_manager': False,
        }],
        condition=IfCondition(slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument('node_prefix', default_value=''),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('extra_params_file', default_value=default_extra_params),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_yaw', default_value='0.0'),

        # --- 定位（slam:=false 才起；slam 模式由 slam_toolbox 取代）---
        Node(
            prefix=node_prefix,
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
            condition=UnlessCondition(slam),
        ),
        Node(
            prefix=node_prefix,
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                # 開機即定位，避免 planner_server 等不到 map 座標系而整批啟動失敗。
                'set_initial_pose': True,
                'initial_pose.x': ParameterValue(init_x, value_type=float),
                'initial_pose.y': ParameterValue(init_y, value_type=float),
                'initial_pose.yaw': ParameterValue(init_yaw, value_type=float),
            }],
            condition=UnlessCondition(slam),
        ),

        # --- 建圖（slam:=true 才起）---
        slam_toolbox_node,
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(AndSubstitution(autostart, slam)),
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox_node,
                start_state='configuring', goal_state='inactive',
                entities=[
                    LogInfo(msg='[EX1] slam_toolbox configure 完成，接著 activate。'),
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            ),
            condition=IfCondition(AndSubstitution(autostart, slam)),
        ),

        # --- 導航 ---
        Node(
            prefix=node_prefix,
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen',
            parameters=common,
        ),
        Node(
            prefix=node_prefix,
            package='nav2_smoother', executable='smoother_server', name='smoother_server',
            output='screen',
            parameters=common,
        ),
        Node(
            prefix=node_prefix,
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen',
            parameters=common,
        ),
        Node(
            prefix=node_prefix,
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen',
            parameters=common,
        ),
        Node(
            prefix=node_prefix,
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen',
            parameters=common,
        ),
        Node(
            prefix=node_prefix,
            package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
            output='screen',
            parameters=common,
        ),

        # --- 生命週期管理器：autostart 時自動把上列節點 configure -> activate ---
        # 兩份、條件互斥，差別只在管不管 map_server/amcl；節點名刻意相同，
        # 讓下游（除錯指令、EX3 supervisor）兩種模式看到的名字一致。
        Node(
            prefix=node_prefix,
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_nav', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': localization_lifecycle_nodes,
            }],
            condition=UnlessCondition(slam),
        ),
        Node(
            prefix=node_prefix,
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_nav', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': nav_lifecycle_nodes,
            }],
            condition=IfCondition(slam),
        ),
    ])
