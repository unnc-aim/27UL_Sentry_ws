"""
Infantry 底盘和云台启动文件

本模块提供步兵机器人底盘控制器和云台控制器的启动配置。

Functions:
    generate_launch_description: 生成 ROS2 launch 描述
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    """
    生成 launch 描述
    
    启动底盘控制器和云台控制器节点，加载配置参数。
    
    Returns:
        LaunchDescription: ROS2 launch 描述对象
    """
    pkg_dir = get_package_share_directory('infantry_controller')
    config_file = os.path.join(pkg_dir, 'config', 'infantry_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Path to the YAML parameter file'
        ),

        # Chassis Controller Node
        Node(
            package='infantry_controller',
            executable='chassis_controller', # Entry point needs to match setup.py
            name='chassis_controller',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            emulate_tty=True
        ),

        # Gimbal Controller Node
        Node(
            package='infantry_controller',
            executable='gimbal_controller', # Entry point needs to match setup.py
            name='gimbal_controller',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            emulate_tty=True
        ),
                # Fire Controller Node
        Node(
            package='infantry_controller',
            executable='fire_controller',
            name='fire_controller',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            emulate_tty=True
        )
    ])
