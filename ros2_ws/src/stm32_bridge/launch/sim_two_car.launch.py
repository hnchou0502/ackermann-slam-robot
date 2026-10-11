"""雙車模擬（V2）：leader 開迴路畫 8 字，follower 用麵包屑＋純追蹤跟隨。零硬體。

  ros2 launch stm32_bridge sim_two_car.launch.py
  選項：
    role:=both          both=單機兩車都起（預設）；leader=只起 car1 那半；follower=只起 car2 那半
                        → 真雙機時：car1 上 role:=leader、car2 上 role:=follower（同 ROS_DOMAIN_ID 互見）
    drive:=false        不自動起 8 字驅動（改用 teleop 手動遙控 leader）
    speed:=0.35         leader 速度 (m/s)
    turn_radius:=0.9    8 字最小轉彎半徑 (m)，必須 > R_min=0.407
    cycles:=1.0         跑幾個 8 字週期（一個週期 ≈ 50.8s @ 預設參數）
    gap0:=1.0           開場時 car2 落後 car1 的距離 (m)
    gap_target:=1.2     跟車目標距離 (m)

節點佈局（namespace 與實車 bringup.launch.py 完全一致，follower 一行都不用改）：
    /car1  kinematic_sim  <- figure8_driver 發 cmd_vel
           v2v_relay      odometry/filtered -> v2v/state @10Hz best-effort
    /car2  kinematic_sim  <- follower 發 cmd_vel
           follower       訂 /car1/v2v/state（跨 namespace，見下）

⚠ follower 的 v2v/state 是**相對名**，在 /car2 底下會解析成 /car2/v2v/state。
   但 leader 發的是 /car1/v2v/state —— 所以這裡明確 remap 到絕對名。
   實車雙機時兩車跑在不同機器上，靠同一個 ROS_DOMAIN_ID 看見彼此，remap 方式相同。

TF：兩車的 /tf 各自 remap 進 namespace（與 bringup.launch.py 同一套方案 B），
    車內 frame 名都叫 odom/base_link，互不相撞。

驗收（另開終端）：
    ros2 run stm32_bridge score_follow --csv /tmp/v2_score.csv --duration 60
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    role = LaunchConfiguration('role')
    drive = LaunchConfiguration('drive')
    speed = LaunchConfiguration('speed')
    turn_radius = LaunchConfiguration('turn_radius')
    cycles = LaunchConfiguration('cycles')
    gap0 = LaunchConfiguration('gap0')
    gap_target = LaunchConfiguration('gap_target')

    # role=both 起兩半（單機）；leader 只起 car1、follower 只起 car2（真雙機各起一半）
    run_leader = IfCondition(PythonExpression(["'", role, "' in ('both', 'leader')"]))
    run_follower = IfCondition(PythonExpression(["'", role, "' in ('both', 'follower')"]))

    car1 = GroupAction([
        PushRosNamespace('car1'),
        Node(
            package='stm32_bridge', executable='kinematic_sim', name='kinematic_sim',
            output='screen', remappings=TF_REMAPPINGS,
        ),
        Node(
            package='stm32_bridge', executable='v2v_relay', name='v2v_relay',
            output='screen', remappings=TF_REMAPPINGS,
        ),
        Node(
            package='stm32_bridge', executable='figure8_driver', name='figure8_driver',
            output='screen', remappings=TF_REMAPPINGS,
            condition=IfCondition(drive),
            parameters=[{
                'speed': ParameterValue(speed, value_type=float),
                'turn_radius': ParameterValue(turn_radius, value_type=float),
                'cycles': ParameterValue(cycles, value_type=float),
            }],
        ),
    ], condition=run_leader)

    car2 = GroupAction([
        PushRosNamespace('car2'),
        Node(
            package='stm32_bridge', executable='kinematic_sim', name='kinematic_sim',
            output='screen', remappings=TF_REMAPPINGS,
        ),
        Node(
            package='stm32_bridge', executable='follower', name='follower',
            output='screen',
            # v2v/state 要跨 namespace 指到 leader；/tf 照舊留在自己的 namespace
            remappings=TF_REMAPPINGS + [('v2v/state', '/car1/v2v/state')],
            parameters=[{
                'gap0': ParameterValue(gap0, value_type=float),
                'gap_target': ParameterValue(gap_target, value_type=float),
            }],
        ),
    ], condition=run_follower)

    return LaunchDescription([
        DeclareLaunchArgument('role', default_value='both'),
        DeclareLaunchArgument('drive', default_value='true'),
        DeclareLaunchArgument('speed', default_value='0.35'),
        DeclareLaunchArgument('turn_radius', default_value='0.9'),
        DeclareLaunchArgument('cycles', default_value='1.0'),
        DeclareLaunchArgument('gap0', default_value='1.0'),
        DeclareLaunchArgument('gap_target', default_value='1.2'),
        car1,
        car2,
    ])
