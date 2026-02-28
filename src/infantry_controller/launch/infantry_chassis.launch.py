import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
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
        )
    ])