#!/usr/bin/env python3
"""
Launch: Nav2 导航 + 正赛控制器 (RMUL 2025)

启动顺序:
  1. Nav2 导航 (rm_navigation_reality_launch.py, slam=True, 在线建图)
  2. 延迟 10s 后启动 competition_controller.py
     controller 内部还会等待 /map + TF + action server 就绪后才进入主循环

slam=True 意味着地图坐标系原点 = 机器人上电位置，
因此 competition_controller.py 中的航点都是相对于出发点的坐标。

Usage:
    ros2 launch scripts/competition_launch.py
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ws_dir = os.path.expanduser('~/sentry_ws')
    log_dir = os.path.join(
        ws_dir, 'log', 'competition',
        datetime.now().strftime('%Y%m%d_%H%M%S'),
    )
    os.makedirs(log_dir, exist_ok=True)

    nav_bringup_dir = get_package_share_directory('pb2025_nav_bringup')
    nav_launch_file = os.path.join(
        nav_bringup_dir, 'launch', 'rm_navigation_reality_launch.py')

    controller_script = os.path.join(ws_dir, 'scripts', 'competition_controller.py')

    # ── Launch arguments ─────────────────────────────────────────
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='ROS log level',
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value='reserve/field_training_latest',
        description='Map world name (prior PCD / initial pose hint)',
    )

    log_env = SetEnvironmentVariable('ROS_LOG_DIR', log_dir)
    log_info = LogInfo(msg=['[COMPETITION] Logs → ', log_dir])

    # ── Nav2 (slam=True, 在线建图; 原点 = 机器人上电位置) ────────
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file),
        launch_arguments={
            'slam': 'True',
            'use_robot_state_pub': 'True',
            'use_rviz': 'False',
            'world': LaunchConfiguration('world'),
            'log_dir': log_dir,
        }.items(),
    )

    # ── 正赛控制器 (延迟 10s 等待 Nav2 初始化) ───────────────────
    controller_node = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[COMPETITION] Nav2 初始化完毕, 启动正赛控制器 ...'),
            ExecuteProcess(
                cmd=[
                    'python3', controller_script,
                    '--ros-args',
                    '--log-level', LaunchConfiguration('log_level'),
                ],
                output='both',
                additional_env={'ROS_LOG_DIR': log_dir},
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_log_level)
    ld.add_action(declare_world)
    ld.add_action(log_env)
    ld.add_action(log_info)
    ld.add_action(nav_cmd)
    ld.add_action(controller_node)
    return ld
