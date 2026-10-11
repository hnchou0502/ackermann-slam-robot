"""半實車模擬（V3）：實車 bag 回放當 leader，follower（模擬車）跟隨。

  ros2 launch stm32_bridge sim_bag_leader.launch.py bag:=/home/haoningrb/bags/washed/map3_washed
  選項：
    bag:=<路徑>       （必填）洗過的 bag——必須是 wash_bag.sh 的產物：
                       只含乾淨 /odometry/filtered、起點已歸零到 (0,0,0°)。
                       ⚠ 不可直接餵原始 bag：其航向帶韌體 yaw 死區失真、起點任意
                       （見 CLAUDE.md 資料地雷 2026-07-11 兩條）。
    gap0:=1.0         開場時 car2 落後 leader 起點的距離 (m)
    gap_target:=1.2   跟車目標距離 (m)

節點佈局（car2 與 sim_two_car.launch.py 完全一致，follower 一行不改）：
    bag play  --remap /odometry/filtered:=/car1/odometry/filtered（rate 固定 1.0，見下）
    /car1  v2v_relay      odometry/filtered -> v2v/state @10Hz best-effort
    /car2  kinematic_sim  <- follower 發 cmd_vel
           follower       訂 /car1/v2v/state

時鐘：全程牆鐘（不用 use_sim_time、不加 --clock）。可行的原因：
  ① v2v_relay 判來源逾時用「收到時刻」且轉發時蓋自己的當下時刻——bag 裡的舊時戳
    到 relay 就被洗掉；② follower 判 leader 逾時也用收到時刻。
  ⚠ 評分器 leader topic 必須指 /car1/v2v/state（吃 relay 蓋的新時戳）；
    指到 bag 直發的 /car1/odometry/filtered 會拿 7/8 的舊時戳配對，全部作廢。

⚠ rate 固定 1.0 不開放：加速回放會讓 leader 位置到達的節奏被壓縮，
   但訊息裡的 twist 速度值仍是原值 -> follower 的速度前饋與現實脫鉤。

驗收（另開終端；bag 播完 relay 會自然停播、follower 逾時停車）：
    ros2 run stm32_bridge score_follow --leader-topic /car1/v2v/state \
        --follower-topic /car2/odometry/filtered --csv /tmp/v3_score.csv --duration <秒>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    gap0 = LaunchConfiguration('gap0')
    gap_target = LaunchConfiguration('gap_target')

    car1 = GroupAction([
        PushRosNamespace('car1'),
        Node(
            package='stm32_bridge', executable='v2v_relay', name='v2v_relay',
            output='screen', remappings=TF_REMAPPINGS,
        ),
    ])

    car2 = GroupAction([
        PushRosNamespace('car2'),
        Node(
            package='stm32_bridge', executable='kinematic_sim', name='kinematic_sim',
            output='screen', remappings=TF_REMAPPINGS,
        ),
        Node(
            package='stm32_bridge', executable='follower', name='follower',
            output='screen',
            remappings=TF_REMAPPINGS + [('v2v/state', '/car1/v2v/state')],
            parameters=[{
                'gap0': ParameterValue(gap0, value_type=float),
                'gap_target': ParameterValue(gap_target, value_type=float),
            }],
        ),
    ])

    # 洗過的 bag 只含 /odometry/filtered；remap 進 /car1 餵 relay。
    # 延遲 3 秒起播，讓 relay/follower 先就位（少掉的只是幾筆靜止開場，無影響）。
    bag_play = TimerAction(period=3.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag,
                 '--remap', '/odometry/filtered:=/car1/odometry/filtered'],
            output='screen',
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='洗過的 bag 目錄（wash_bag.sh 產物）'),
        DeclareLaunchArgument('gap0', default_value='1.0'),
        DeclareLaunchArgument('gap_target', default_value='1.2'),
        car1,
        car2,
        bag_play,
    ])
