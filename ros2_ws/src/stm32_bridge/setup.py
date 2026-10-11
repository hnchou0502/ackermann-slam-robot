import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stm32_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haoningrb',
    maintainer_email='180163860+hnchou0502@users.noreply.github.com',
    description='STM32 自走車 ROS2 橋接（讀 $FBK，發 /odom + /imu/data，收 /cmd_vel；含 EKF bringup、V2V 轉播與雙車模擬）。',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge = stm32_bridge.bridge_node:main',
            'v2v_relay = stm32_bridge.v2v_relay:main',
            # V2 雙車模擬與驗收工具（零硬體）
            'kinematic_sim = stm32_bridge.kinematic_sim:main',
            'follower = stm32_bridge.follower:main',
            'figure8_driver = stm32_bridge.figure8_driver:main',
            'score_follow = stm32_bridge.score_follow:main',
            # CM2 即時合圖（2026-07-21）：訂兩張 /map 自動對齊融合出 merged_map
            'map_merge_live = stm32_bridge.map_merge_live:main',
            # EX2 自主探索（2026-08-11）：訂 /map 找未探索邊界，經 Nav2 一個個開過去。
            # ⚠ 實跑一律經 EX3 的 supervisor 啟動，不要單獨起（會繞過急停鏈）。
            'frontier_explorer = stm32_bridge.frontier_explorer:main',
        ],
    },
)
