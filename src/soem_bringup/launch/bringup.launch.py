"""
SOEM EtherCAT 主站启动文件

本模块提供 SOEM EtherCAT 主站的启动配置，用于与步兵机器人的 EtherCAT 设备通信。

Functions:
    generate_launch_description: 生成 ROS2 launch 描述
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    """
    生成 launch 描述
    
    启动 SOEM 后端节点，配置网络接口、CPU 亲和性和配置文件路径。
    
    Returns:
        LaunchDescription: ROS2 launch 描述对象
    """
    config_file = os.path.join(
        get_package_share_directory('soem_bringup'),
        'config',
        'config_sentry.yaml'
    )

    return LaunchDescription([
        Node(
            package='soem_wrapper',
            executable='soem_backend',
            name='soem_backend',
            parameters=[{
                'interface': "enp2s0",
                'rt_cpu': 0,
                'non_rt_cpus': "1,2,3,4,5,6,7,8,9,10,11",
                'config_file': config_file
            }],
            output='screen'
        )
    ])
