#!/usr/bin/env python3
"""
Launch file: start Nav2 navigation (SLAM mode), wait 30s, run simple nav test.
Records detailed logs to ~/sentry_ws/log/simple_nav_test/<timestamp>/.

Usage:
    ros2 launch scripts/test_simple_nav_launch.py
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
        ws_dir, 'log', 'simple_nav_test',
        datetime.now().strftime('%Y%m%d_%H%M%S'),
    )
    os.makedirs(log_dir, exist_ok=True)

    nav_bringup_dir = get_package_share_directory('pb2025_nav_bringup')
    nav_launch_file = os.path.join(
        nav_bringup_dir, 'launch', 'rm_navigation_reality_launch.py')

    test_script = os.path.join(ws_dir, 'scripts', 'test_simple_nav.py')

    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='ROS log level',
    )

    log_env = SetEnvironmentVariable('ROS_LOG_DIR', log_dir)
    log_info = LogInfo(msg=['[TEST] Logs: ', log_dir])

    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file),
        launch_arguments={
            'slam': 'True',
            'use_robot_state_pub': 'True',
            'use_rviz': 'False',
            'log_dir': log_dir,
        }.items(),
    )

    test_node = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[TEST] 30s elapsed, starting simple nav test...'),
            ExecuteProcess(
                cmd=[
                    'python3', test_script,
                    '--ros-args', '--log-level', LaunchConfiguration('log_level'),
                ],
                output='both',
                additional_env={'ROS_LOG_DIR': log_dir},
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_log_level)
    ld.add_action(log_env)
    ld.add_action(log_info)
    ld.add_action(nav_cmd)
    ld.add_action(test_node)
    return ld
