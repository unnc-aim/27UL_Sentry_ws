"""Isolated Gazebo + AMCL + existing omni controller. Never launches hardware."""
import os
import json
from pathlib import Path
import sys

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).parent))
from scenario import make_arena, make_robot


def setup(context):
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('IGN_PARTITION') != 'sentry-frame-sim':
        raise RuntimeError('Use the documented isolated ROS_DOMAIN_ID=87 and IGN_PARTITION=sentry-frame-sim')
    directory = Path(context.launch_configurations['output_dir'])
    make_arena(directory)
    (directory/'result.json').write_text(json.dumps({'passed': False, 'status': 'running'})+'\n')
    (directory/'run_config.json').write_text(json.dumps({
        'spawn_yaw': float(context.launch_configurations['spawn_yaw']),
        'nav_spin_speed': float(context.launch_configurations['nav_spin_speed']),
        'robot_model': 'pb2025_robot_description/simulation_robot',
    }, indent=2)+'\n')
    sdf, urdf = make_robot(share('pb2025_robot_description'))
    (directory/'robot.sdf').write_text(sdf)
    ns = 'coordinate_robot'
    bridge = [dict(ros_topic_name='/clock', gz_topic_name='/clock',
                   ros_type_name='rosgraph_msgs/msg/Clock', gz_type_name='ignition.msgs.Clock', direction='GZ_TO_ROS')]
    for topic, gz_topic, ros_type, gz_type in [
        ('ground_truth', f'/{ns}/odometry', 'nav_msgs/msg/Odometry', 'Odometry'),
        ('joint_states', f'/world/default/model/{ns}/joint_state', 'sensor_msgs/msg/JointState', 'Model'),
        ('scan', f'/world/default/model/{ns}/link/front_rplidar_a2/sensor/front_rplidar_a2/scan', 'sensor_msgs/msg/LaserScan', 'LaserScan'),
    ]:
        bridge.append(dict(ros_topic_name=f'/{ns}/{topic}', gz_topic_name=gz_topic,
                           ros_type_name=ros_type, gz_type_name='ignition.msgs.'+gz_type, direction='GZ_TO_ROS'))
    (directory/'bridge.yaml').write_text(yaml.safe_dump(bridge))
    params = yaml.safe_load((Path(__file__).parent/'params.yaml').read_text())
    params['map_server']['ros__parameters']['yaml_filename'] = str(directory/'map.yaml')
    # Reuse the workspace's tested omni controller and BT plugin set.
    source = yaml.safe_load((Path(share('pb2025_nav_bringup'))/'config/simulation/nav2_params.yaml').read_text())
    params['bt_navigator']['ros__parameters']['plugin_lib_names'] = source['bt_navigator']['ros__parameters']['plugin_lib_names']
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(Path(__file__).parent/'navigate.xml')
    params['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'] = str(Path(__file__).parent/'navigate_through.xml')
    params.pop('/**', None)
    def sim_time(tree):
        for key, value in tree.items():
            if key == 'ros__parameters':
                value['use_sim_time'] = True
            elif isinstance(value, dict):
                sim_time(value)
    sim_time(params)
    (directory/'params.yaml').write_text(yaml.safe_dump({ns: params}))
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    def node(pkg, executable, name=None, **kwargs):
        return Node(package=pkg, executable=executable, name=name, namespace=ns,
                    output='log', remappings=remaps, **kwargs)
    actions = [
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '--headless-rendering', str(directory/'arena.sdf')], output='log'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-g'],
                       condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        node('ros_gz_sim', 'create', arguments=['-file', str(directory/'robot.sdf'), '-name', ns,
             '-x', '-2.5', '-y', '0', '-z', '.25', '-Y', context.launch_configurations['spawn_yaw']]),
        node('robot_state_publisher', 'robot_state_publisher', parameters=[{'use_sim_time': True, 'robot_description': urdf}]),
        node('ros_gz_bridge', 'parameter_bridge', parameters=[{'config_file': str(directory/'bridge.yaml')}]),
        node('rmoss_gz_base', 'rmua19_robot_base', parameters=[str(Path(share('rmu_gazebo_simulator'))/'config/base_params.yaml'), {'robot_name': ns, 'use_sim_time': True}]),
        ExecuteProcess(cmd=['python3', str(Path(__file__).parent/'bridge.py'), '--ros-args',
                           '-r', '__ns:=/'+ns, '-r', '/tf:=tf', '-r', '/tf_static:=tf_static',
                           '-p', 'use_sim_time:=true', '-p',
                           'nav_spin_speed:='+context.launch_configurations['nav_spin_speed']], output='log'),
    ]
    nodes = [('nav2_map_server', 'map_server'), ('nav2_amcl', 'amcl'),
             ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
             ('nav2_bt_navigator', 'bt_navigator')]
    for pkg, exe in nodes:
        extra = [('cmd_vel', 'nav_cmd_vel')] if exe == 'controller_server' else []
        actions.append(Node(package=pkg, executable=exe, name=exe, namespace=ns,
                            parameters=[str(directory/'params.yaml')], remappings=remaps+extra, output='log'))
    actions.append(node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager',
                        parameters=[{'use_sim_time': True, 'autostart': True,
                                     'node_names': [exe for _, exe in nodes]}]))
    actions.append(ExecuteProcess(cmd=['python3', str(Path(__file__).parent/'mission.py'),
        '--output', str(directory/'result.json'), '--landmarks', str(directory/'landmarks.yaml'),
        '--ros-args', '-r', '__ns:=/'+ns, '-r', '/tf:=tf', '-r', '/tf_static:=tf_static', '-p', 'use_sim_time:=true'], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the Gazebo GUI alongside the simulation server'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.2'),
        DeclareLaunchArgument('nav_spin_speed', default_value='0.0'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/sentry-frame-sim/run'),
        OpaqueFunction(function=setup),
    ])
