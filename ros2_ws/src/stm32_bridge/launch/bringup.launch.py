"""bringup：stm32_bridge（讀+送+IMU）+ robot_localization EKF + 光達（見 lidar_type）。

單車（不變的原跑法）：
  ros2 launch stm32_bridge bringup.launch.py

雙車（V0，2026-07-10 起）：兩台各自帶 namespace，同一個 ROS_DOMAIN_ID。
  car1:  ros2 launch stm32_bridge bringup.launch.py namespace:=car1 use_v2v_relay:=true
  car2:  ros2 launch stm32_bridge bringup.launch.py namespace:=car2

選項：
  namespace:=''       命名空間（預設空＝根，單車行為與 V0 之前完全相同）
  enable_cmd:=false   純唯讀（不送 $CMD，車絕不會動）
  use_ekf:=false      只跑橋接、不跑 EKF；此時請一併 publish_tf:=true（讓橋接自己發 TF）
  publish_tf:=true    橋接自己發 odom->base_link（預設 false：交給 EKF 發）
  use_lidar:=false    不啟動光達 driver（無光達的機器上跑純橋接時用）
  lidar_type:=c1     光達型號（**預設 c1**＝car1 現役 RPLIDAR C1，與 laser_x/z/yaw 預設值
                      同屬一套硬體——2026-07-21 Codex F1 修正，舊預設 ydlidar 已退役）：
                      ydlidar＝退役的 YDLIDAR X4（復用須手動帶 laser_x:=0.123 laser_z:=0.185 laser_yaw:=0）；
                      rplidar＝car2 的 RPLIDAR A1M8（rplidar_ros 的 rplidar_composition，
                      參數 config/rplidar_a1.yaml——不用官方 rplidar.launch.py，它參數寫死）；
                      c1＝car1 的 RPLIDAR C1（2026-07-20 起，取代 X4；sllidar_ros2 的
                      sllidar_node，參數 config/sllidar_c1.yaml——⚠ 460800 baud，
                      apt 版 rplidar_ros 不支援 C1，要 source build sllidar_ros2）
  laser_x / laser_z   base_link->laser_frame 靜態 TF 的安裝位置（公尺）。
                      預設＝car1 **C1** 捲尺實測值（2026-07-20：x=0.19、z=0.155）。
                      ⚠ 舊 YDLIDAR X4 的裝位是 x=0.123、z=0.185、yaw=0——X4 已退役，
                        若要復用（lidar_type:=ydlidar）**必須三個值一起手動帶回**，
                        否則地圖會歪掉且完全不報錯。
  laser_yaw           光達安裝朝向（弧度）。預設 3.14159265（180°＝零度朝車尾）：
                      car1 C1 與 car2 A1M8 皆為反裝，兩車差分定位實測一致
                      （car2 2026-07-17、car1 2026-07-20，訊號/雜訊比 79 倍）
  use_v2v_relay:=true 啟動 v2v_relay（leader 車用；把 odometry/filtered 轉播成 v2v/state）

預設：use_ekf=true + publish_tf=false → 橋接出 /odom 與 /imu/data 兩路，
   EKF 融合後出 /odometry/filtered 並發 odom->base_link TF。
   光達 driver 出 /scan(frame_id=laser_frame)，靜態 TF base_link->laser_frame。

TF 樹：map(SLAM 待接) -> odom(EKF) -> base_link(EKF) -> laser_frame(本檔靜態 TF)。

命名空間化（V0；設計＝docs/design/v2v_architecture.md §3 方案 B）
  - 節點與 topic 進 namespace（/car1/odom、/car1/scan…）；
  - **整棵 TF 樹**也 remap 進 namespace（/car1/tf、/car1/tf_static），
    但**車內 frame 名完全不動**（仍叫 odom / base_link / laser_frame）。
    兩車的 TF 樹只需「不相撞」，不需「互通」——follower 只消費 leader 的 v2v/state。
    這繞開了 robot_localization 的 tf_prefix 參數失效問題（issue #843）。
  - 兩份 config yaml 的頂層 key 都是 `/**`，否則進 namespace 後參數會靜默不載入。

⚠ 光達參數改用本套件的 config/ydlidar_x4.yaml（V0 起），launch 解析期不再相依
   ydlidar_ros2_driver 的 share 目錄——無光達的機器也能安全跑 use_lidar:=false。
   （driver 執行檔本身仍需安裝才能 use_lidar:=true。）
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

# 把整棵 TF 樹推進 namespace：'/tf' 是絕對名（不受 namespace 影響），
# remap 成相對名 'tf' 後才會解析成 /<namespace>/tf。namespace 為空時解析回 /tf（原行為）。
TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

# robot_localization 的 diagnostic_updater 同樣硬寫絕對名 '/diagnostics'，
# 不 remap 的話兩車會把診斷訊息灌進同一個 topic（2026-07-10 V0 實測確認）。
# 註：/rosout 與 /parameter_events 是 ROS2 內建的全域 topic，設計上就不進 namespace，不處理。
EKF_REMAPPINGS = TF_REMAPPINGS + [('/diagnostics', 'diagnostics')]


def generate_launch_description():
    pkg = get_package_share_directory('stm32_bridge')
    ekf_params = os.path.join(pkg, 'config', 'ekf.yaml')
    ydlidar_params = os.path.join(pkg, 'config', 'ydlidar_x4.yaml')
    rplidar_params = os.path.join(pkg, 'config', 'rplidar_a1.yaml')
    sllidar_c1_params = os.path.join(pkg, 'config', 'sllidar_c1.yaml')

    namespace = LaunchConfiguration('namespace')
    enable_cmd = LaunchConfiguration('enable_cmd')
    use_ekf = LaunchConfiguration('use_ekf')
    publish_tf = LaunchConfiguration('publish_tf')
    use_lidar = LaunchConfiguration('use_lidar')
    lidar_type = LaunchConfiguration('lidar_type')
    laser_x = LaunchConfiguration('laser_x')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')
    use_v2v_relay = LaunchConfiguration('use_v2v_relay')
    cmd_reversal_dwell_s = LaunchConfiguration('cmd_reversal_dwell_s')
    cmd_max_accel = LaunchConfiguration('cmd_max_accel')

    # PushRosNamespace('') 會把節點推進根命名空間，行為上等同不推；
    # 但為了讓單車路徑與 V0 之前「逐位元相同」，空字串時完全不執行這個 action。
    push_ns = PushRosNamespace(
        namespace=namespace,
        condition=IfCondition(PythonExpression(["'", namespace, "' != ''"])),
    )

    # EX3 安全機制 b「親死子亡」：supervisor 以 'setpriv --pdeathsig=SIGKILL' 當前綴
    # 啟動每個節點，這樣連 supervisor 本身被 SIGKILL，子行程也會被 kernel 一起收掉，
    # 不會留下孤兒繼續發 /cmd_vel。預設空字串＝原本的跑法完全不受影響。
    node_prefix = LaunchConfiguration('node_prefix')

    return LaunchDescription([
        DeclareLaunchArgument('node_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('enable_cmd', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        # 預設 c1＝car1 現役光達（2026-07-21 Codex F1：預設驅動與預設 TF 必須屬同一硬體。
        # 先前預設 ydlidar（退役 X4）配 C1 的 TF 預設值＝自相矛盾組合，裸跑會靜默出錯圖）。
        # 對照測試：test/test_launch_defaults.py 鎖住這組配對。
        DeclareLaunchArgument('lidar_type', default_value='c1'),
        # 預設＝car1 C1 實測裝位（2026-07-20 捲尺：x=0.19 後軸往前、z=0.155 離地）。
        # 舊 X4 是 x=0.123 / z=0.185 / yaw=0（2026-07-08 量），已退役——見檔頭警告。
        DeclareLaunchArgument('laser_x', default_value='0.19'),
        DeclareLaunchArgument('laser_z', default_value='0.155'),
        # 180°：car1 C1 與 car2 A1M8 都是零度朝車尾（差分定位實測，見檔頭）。
        DeclareLaunchArgument('laser_yaw', default_value='3.14159265'),
        DeclareLaunchArgument('use_v2v_relay', default_value='false'),
        # bridge 指令整形（EX4 限速，2026-09-24）。預設 0＝關閉＝原跑法不變；
        # explore.launch.py 會打開。細節見 protocol.CmdShaper。
        DeclareLaunchArgument('cmd_reversal_dwell_s', default_value='0.0'),
        DeclareLaunchArgument('cmd_max_accel', default_value='0.0'),

        GroupAction([
            push_ns,

            Node(
                prefix=node_prefix,
                package='stm32_bridge', executable='bridge', name='stm32_bridge',
                output='screen',
                parameters=[{
                    'enable_cmd': ParameterValue(enable_cmd, value_type=bool),
                    'publish_tf': ParameterValue(publish_tf, value_type=bool),
                    'reversal_dwell_s': ParameterValue(cmd_reversal_dwell_s, value_type=float),
                    'max_linear_accel': ParameterValue(cmd_max_accel, value_type=float),
                }],
                remappings=TF_REMAPPINGS,
            ),
            Node(
                prefix=node_prefix,
                package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                output='screen',
                parameters=[ekf_params],
                remappings=EKF_REMAPPINGS,
                condition=IfCondition(use_ekf),
            ),

            # 光達 driver（依 lidar_type 二選一）：/scan（frame_id=laser_frame）。
            # ydlidar（car1 X4 ~7.9Hz）：port 與 fixed_resolution 的正確值在 config/ydlidar_x4.yaml。
            #   (⚠ fixed_resolution:false × slam_toolbox 點數硬檢查是已知雷區，T10 已複驗可用)
            Node(
                prefix=node_prefix,
                package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                name='ydlidar_ros2_driver_node', output='screen',
                parameters=[ydlidar_params],
                remappings=TF_REMAPPINGS,
                condition=IfCondition(PythonExpression(
                    ["'", lidar_type, "' == 'ydlidar' and '", use_lidar, "' == 'true'"])),
            ),
            # rplidar（car2 A1M8 ~7.3Hz，2026-07-17 驗收）：參數在 config/rplidar_a1.yaml。
            Node(
                prefix=node_prefix,
                package='rplidar_ros', executable='rplidar_composition',
                name='rplidar_node', output='screen',
                parameters=[rplidar_params],
                remappings=TF_REMAPPINGS,
                condition=IfCondition(PythonExpression(
                    ["'", lidar_type, "' == 'rplidar' and '", use_lidar, "' == 'true'"])),
            ),
            # c1（car1 RPLIDAR C1，10.0Hz／16m／720點，2026-07-20 驗收）：參數在 config/sllidar_c1.yaml。
            # ⚠ 驅動是 sllidar_ros2（Slamtec 官方 source build），不是上面 rplidar 分支的 apt 套件。
            Node(
                prefix=node_prefix,
                package='sllidar_ros2', executable='sllidar_node',
                name='sllidar_node', output='screen',
                parameters=[sllidar_c1_params],
                remappings=TF_REMAPPINGS,
                condition=IfCondition(PythonExpression(
                    ["'", lidar_type, "' == 'c1' and '", use_lidar, "' == 'true'"])),
            ),

            # 靜態 TF base_link -> laser_frame（預設＝car1 捲尺實測；car2 用 laser_x/laser_z 覆蓋）。
            # car1 yaw=0 已於 T10 前置複驗（正前放物，/scan 最近角落在 +2.9°≈0°）；car2 裝機時同法複驗。
            Node(
                prefix=node_prefix,
                package='tf2_ros', executable='static_transform_publisher',
                name='static_tf_base_to_laser', output='log',
                arguments=[
                    '--x', laser_x, '--y', '0.0', '--z', laser_z,
                    '--yaw', laser_yaw, '--pitch', '0.0', '--roll', '0.0',
                    '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
                ],
                remappings=TF_REMAPPINGS,
            ),

            # V2V 轉播（leader 車才需要）：odometry/filtered -> v2v/state @10Hz best-effort。
            Node(
                prefix=node_prefix,
                package='stm32_bridge', executable='v2v_relay', name='v2v_relay',
                output='screen',
                remappings=TF_REMAPPINGS,
                condition=IfCondition(use_v2v_relay),
            ),
        ]),
    ])
