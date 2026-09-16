"""Diagnostic navigation in domain 88 only. No hardware actuator or RC nodes."""
import os
import json
import math
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
    params['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'] = .04
    params['livox_ros_driver2']['ros__parameters']['user_config_path'] = str(bringup/'config/reality/mid360_user_config.json')
    params['fake_vel_transform']['ros__parameters'].update(
        output_cmd_vel_topic='/static_acceptance/cmd_vel',
        cmd_spin_topic='/static_acceptance/cmd_spin', init_spin_speed=0.0)
    # Let the mapper publish its estimated map->odom correction; do not freeze it.
    params['slam_toolbox']['ros__parameters'].update(
        transform_publish_period=.05, minimum_travel_distance=0.0,
        minimum_travel_heading=0.0)  # Gimbal sweeps while chassis stays stationary.
    # Static diagnostics must not save/overwrite a real map on shutdown.
    params['point_lio']['ros__parameters'].setdefault('pcd_save', {})['pcd_save_en'] = False
    saved_map = os.environ.get('SENTRY_ACCEPTANCE_SAVED_MAP')
    if saved_map:
        saved_map = str(Path(saved_map).resolve(strict=True))
        localization = yaml.safe_load((Path(__file__).parent.parent/'coordinate_sim/params.yaml').read_text())
        params['map_server'] = localization['map_server']
        params['map_server']['ros__parameters'].update(yaml_filename=saved_map, use_sim_time=False)
        params['amcl'] = localization['amcl']
        params['amcl']['ros__parameters'].update(use_sim_time=False, scan_topic='obstacle_scan',
            update_min_a=0.0, update_min_d=0.0, laser_min_range=.3, laser_max_range=10.0,
            min_particles=1000, max_particles=5000, max_beams=60,
            recovery_alpha_fast=0.0, recovery_alpha_slow=0.0)
    directory = Path(tempfile.mkdtemp(prefix='sentry-static-'))
    config = directory/'params.yaml'
    config.write_text(yaml.safe_dump(params).replace("<robot_namespace>", ""))
    macro = XMLMacro4sdf()
    macro.set_xml_file(str(Path(share('pb2025_robot_description'))/'resource/xmacro/pb2025_sentry_robot.sdf.xmacro'))
    macro.generate()
    generator = UrdfGenerator()
    generator.parse_from_sdf_string(macro.to_string())
    # The original model splits one physical yaw across two virtual yaw joints.
    # Collapse the translation-only intermediate chain in this diagnostic URDF;
    # publish the measured physical yaw once, without inventing a second encoder.
    urdf = ET.fromstring(generator.to_string())
    chain = [urdf.find("joint[@name='" + name + "']") for name in
             ['gimbal_yaw_odom_joint', 'gimbal_pitch_odom_joint', 'gimbal_yaw_joint']]
    if any(j is None for j in chain):
        raise RuntimeError('Unexpected yaw chain; re-audit robot geometry')
    origins = [j.find('origin') for j in chain]
    if any(o.get('rpy') != '0 0 0' for o in origins):
        raise RuntimeError('Yaw chain has rotated origins; re-audit composition')
    xyz = [sum(float(o.get('xyz').split()[i]) for o in origins) for i in range(3)]
    chain[-1].find('parent').set('link', chain[0].find('parent').get('link'))
    chain[-1].find('origin').set('xyz', ' '.join(map(str, xyz)))
    for joint in chain[:-1]:
        link_name = joint.find('child').get('link')
        urdf.remove(joint)
        urdf.remove(urdf.find("link[@name='" + link_name + "']"))
    # Operator confirmed MID360 rotates with yaw, unlike the upstream chassis mount.
    # Preserve nominal zero-yaw installation geometry; numerical extrinsics remain
    # subject to physical acceptance, not inferred from the parent correction.
    lidar_joint = urdf.find("joint[@name='front_livox_joint']")
    if lidar_joint is None or lidar_joint.find('parent').get('link') != 'chassis':
        raise RuntimeError('Unexpected lidar mount; re-audit model')
    if chain[-1].find('parent').get('link') != 'chassis':
        raise RuntimeError('Unexpected yaw parent; re-audit model')
    origin = lidar_joint.find('origin')
    lidar_xyz = [float(v) for v in origin.get('xyz').split()]
    origin.set('xyz', ' '.join(str(lidar_xyz[i]-xyz[i]) for i in range(3)))
    lidar_joint.find('parent').set('link', 'gimbal_yaw')
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
