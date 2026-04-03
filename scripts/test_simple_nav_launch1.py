#!/usr/bin/env python3
"""
Launch file: run stationary function test (no Nav2 / no movement required).
Records detailed logs to ~/sentry_ws/log/simple_nav_test/<timestamp>/.

Tests gimbal scan, auto-aim, and chassis spin (小陀螺) toggle + data flow.

Usage:
    ros2 launch scripts/test_simple_nav_launch1.py
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ws_dir = os.path.expanduser('~/sentry_ws')
    log_dir = os.path.join(
        ws_dir, 'log', 'simple_nav_test',
        datetime.now().strftime('%Y%m%d_%H%M%S'),
    )
    os.makedirs(log_dir, exist_ok=True)

    test_script = os.path.join(ws_dir, 'scripts', 'test_stationary_func.py')

    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='ROS log level',
    )

    log_env = SetEnvironmentVariable('ROS_LOG_DIR', log_dir)
    log_info = LogInfo(msg=['[TEST] Logs: ', log_dir])

    test_node = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[TEST] Starting stationary function test...'),
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
    ld.add_action(test_node)
    return ld
