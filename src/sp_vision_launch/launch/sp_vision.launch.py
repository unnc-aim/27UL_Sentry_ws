import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='sentry.yaml',
        description='Config file name (e.g., sentry.yaml, infantry.yaml)'
    )

    # Get sp_vision_launch share directory (contains symlinked configs)
    sp_vision_launch_dir = get_package_share_directory('sp_vision_launch')
    
    # Derive sp_vision binary path from install prefix
    launch_share_dir = Path(sp_vision_launch_dir)
    install_prefix = launch_share_dir.parents[2]
    binary_path = install_prefix / 'sp_vision' / 'bin' / 'sentry'

    if not binary_path.exists():
        raise RuntimeError(f'sp_vision binary not found: {binary_path}')

    # Config path uses symlink in sp_vision_launch -> points to source configs
    config_path = PathJoinSubstitution([
        sp_vision_launch_dir,
        'configs',
        LaunchConfiguration('config'),
    ])
    
    # Working directory for relative asset paths
    sp_vision_share_dir = install_prefix / 'sp_vision' / 'share' / 'sp_vision'

    return LaunchDescription([
        config_arg,
        ExecuteProcess(
            cmd=[str(binary_path), config_path],
            cwd=str(sp_vision_share_dir),
            output='screen',
        ),
    ])
