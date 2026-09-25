"""Diagnostic navigation in domain 88 only. No hardware actuator or RC nodes."""
import os
import json
import math
import runpy
import sys
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Static acceptance requires ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1')
    diagnostic_radius = 0.20  # User-confirmed real robot radius; matches production Nav2.
    bringup = Path(share('pb2025_nav_bringup'))
    params = yaml.safe_load((bringup/'config/reality/nav2_params.yaml').read_text())
    for costmap in ('local_costmap', 'global_costmap'):
        params[costmap][costmap]['ros__parameters']['robot_radius'] = diagnostic_radius
        params[costmap][costmap]['ros__parameters']['always_send_full_costmap'] = True
    params['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'] = .04
    # At 0.15 m/s, steering and bounded gaze alignment precede translation.
    # Keep a progress deadline, but do not require 0.30 m during that startup.
    params['controller_server']['ros__parameters']['progress_checker'].update(
        required_movement_radius=.05, movement_time_allowance=20.0)
    # Keep publishing zero on collision while the 1 Hz planner updates its path.
    params['controller_server']['ros__parameters']['failure_tolerance'] = 2.0
    params['livox_ros_driver2']['ros__parameters']['user_config_path'] = str(bringup/'config/reality/mid360_user_config.json')
    params['fake_vel_transform']['ros__parameters'].update(
        output_cmd_vel_topic='/static_acceptance/cmd_vel',
        cmd_spin_topic='/static_acceptance/cmd_spin', init_spin_speed=0.0, translation_scale=1.0)
    # Let the mapper publish its estimated map->odom correction; do not freeze it.
    params['slam_toolbox']['ros__parameters'].update(
        transform_publish_period=.05, minimum_travel_distance=0.0,
        minimum_travel_heading=0.0)  # Gimbal sweeps while chassis stays stationary.
    # Static diagnostics must not save/overwrite a real map on shutdown.
    params['point_lio']['ros__parameters'].setdefault('pcd_save', {})['pcd_save_en'] = False
    saved_map = os.environ.get('SENTRY_ACCEPTANCE_SAVED_MAP')
    if saved_map:
        saved_map = str(Path(saved_map).resolve(strict=True))
        params['map_server']['ros__parameters'].update(yaml_filename=saved_map, use_sim_time=False)
        params['amcl']['ros__parameters'].update(use_sim_time=False, set_initial_pose=False)
    directory = Path(tempfile.mkdtemp(prefix='sentry-static-'))
    config = directory/'params.yaml'
    config.write_text(yaml.safe_dump(params).replace("<robot_namespace>", ""))
    macro = XMLMacro4sdf()
    macro.set_xml_file(str(Path(share('pb2025_robot_description'))/'resource/xmacro/pb2025_sentry_robot.sdf.xmacro'))
    macro.generate()
    generator = UrdfGenerator()
    generator.parse_from_sdf_string(macro.to_string())
    # Use the same measured-yaw geometry as the production real robot.
    real_sentry_urdf = runpy.run_path(str(
        Path(share('pb2025_robot_description'))/'launch/robot_description_launch.py'
    ))['real_sentry_urdf']
    urdf = ET.fromstring(real_sentry_urdf(generator.to_string()))
    lidar_joint = urdf.find("joint[@name='front_livox_joint']")
    origin = lidar_joint.find('origin')
    calibration_path = os.environ.get('SENTRY_ACCEPTANCE_LIDAR_CALIBRATION')
    if calibration_path:
        calibration = json.loads(Path(calibration_path).read_text())
        delta = calibration['correction_xy_m']
        if (not calibration.get('stationary_confirmed') or
            calibration.get('status') == 'invalid_do_not_apply' or
            len(delta) != 2 or not all(math.isfinite(v) for v in delta) or
            math.hypot(*delta) > .15 or calibration['fit_rms_m'] > .015):
            raise RuntimeError('Unusable candidate lidar calibration')
        corrected = [float(v) for v in origin.get('xyz').split()]
        corrected[0] += delta[0]; corrected[1] += delta[1]
        origin.set('xyz', ' '.join(map(str, corrected)))

    # No joint_state_publisher: absent measured joints must remain absent.
    actions = [ExecuteProcess(cmd=['python3', str(Path(__file__).parent/'yaw_feedback.py')], output='log'), Node(package='robot_state_publisher', executable='robot_state_publisher',
                    parameters=[{'robot_description': ET.tostring(urdf, encoding='unicode'), 'use_sim_time': False}], output='log'),
               Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                    name='livox_ros_driver2', parameters=[str(config)], output='log')]
    # Reuse navigation algorithms only. No joy, competition, Hub, motor or fire launch.
    # Online mapping is a local diagnostic frame, NOT prebuilt-map localization acceptance.
    for package, executable, name, overrides, remaps in [
        ('point_lio', 'pointlio_mapping', 'point_lio',
         {'prior_pcd.enable': False, 'pcd_save.pcd_save_en': False}, []),
        ('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node', 'pointcloud_to_laserscan',
         {}, [('cloud_in', 'terrain_map_ext'), ('scan', 'obstacle_scan')]),
        ('slam_toolbox', 'sync_slam_toolbox_node', 'slam_toolbox', {}, []),
    ]:
        if saved_map and package == 'slam_toolbox':
            continue  # AMCL is the only map->odom source in saved-map mode.
        actions.append(Node(package=package, executable=executable, name=name,
                            parameters=[str(config), overrides], remappings=remaps, output='log'))
    if saved_map:
        for package, executable in [('nav2_map_server','map_server'),('nav2_amcl','amcl')]:
            actions.append(Node(package=package, executable=executable, name=executable,
                                parameters=[str(config)], output='log'))
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', parameters=[{'autostart':True,
                'use_sim_time':False, 'node_names':['map_server','amcl']}], output='log'))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup/'launch/navigation_launch.py')),
        launch_arguments={'namespace': '', 'params_file': str(config),
                          'use_sim_time': 'False', 'use_composition': 'False',
                          'autostart': 'true', 'use_respawn': 'False'}.items()))
    return LaunchDescription(actions)
