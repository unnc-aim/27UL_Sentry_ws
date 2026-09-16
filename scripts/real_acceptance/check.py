#!/usr/bin/env python3
"""Read-only ROS preflight. Never authorizes motion or publishes commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--output', default='/tmp/sentry-real-acceptance.json')
    parser.add_argument('--isolated', action='store_true')
    args = parser.parse_args()
    if args.isolated and os.environ.get('ROS_DOMAIN_ID') != '88':
        parser.error('--isolated requires ROS_DOMAIN_ID=88')
    if not 2 <= args.seconds <= 120:
        parser.error('--seconds must be between 2 and 120')
    root = Path(__file__).resolve().parents[2]
    protected = root / 'src/universal_controller'
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in protected.rglob('*') if p.is_file() and p.suffix in ('.cpp', '.hpp', '.yaml', '.yml')}
    rclpy.init()
    node = rclpy.create_node('real_acceptance_observer', enable_rosout=False,
                             start_parameter_services=False)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    received = {}
    subscriptions = {}
    sensor_topics = ['/livox/imu', '/livox/lidar', '/odometry', '/joint_states' if args.isolated else '/serial/gimbal_joint_state']
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        for topic, types in node.get_topic_names_and_types():
            if topic in sensor_topics and topic not in subscriptions and len(types) == 1:
                subscriptions[topic] = node.create_subscription(
                    get_message(types[0]), topic,
                    lambda msg, name=topic: received.update({name: time.monotonic()}),
                    qos_profile_sensor_data)
        rclpy.spin_once(node, timeout_sec=0.1)
    topics = dict(node.get_topic_names_and_types())
    checks = []
    def check(name, ok, detail):
        checks.append(dict(name=name, passed=bool(ok), detail=detail))
    interfaces = {}
    command_topic = '/static_acceptance/cmd_vel' if args.isolated else '/cmd_vel'
    inspected = [command_topic, '/map'] if args.isolated else [command_topic, '/cmd_spin', '/map', '/referee/common/game_status']
    for topic in inspected:
        pubs = node.get_publishers_info_by_topic(topic)
        subs = node.get_subscriptions_info_by_topic(topic)
        interfaces[topic] = {
            'publishers': [dict(node=p.node_name, type=p.topic_type, qos=str(p.qos_profile)) for p in pubs],
            'subscribers': [dict(node=p.node_name, type=p.topic_type, qos=str(p.qos_profile)) for p in subs]}
        check(topic + ':publisher', len(pubs) > 0, f'{len(pubs)} publishers; presence does not prove fresh data')
        if topic == command_topic:
            check('cmd_vel:type', bool(pubs) and (args.isolated or bool(subs)) and
                  all(p.topic_type == 'geometry_msgs/msg/TwistStamped' for p in pubs + subs),
                  'All endpoints must use the currently expected Hub TwistStamped interface')
    for parent, child in [('map', 'odom'), ('odom', 'base_footprint'), ('base_footprint', 'gimbal_yaw')]:
        try:
            tf = buffer.lookup_transform(parent, child, Time())
            check(parent + '->' + child, True, {'stamp_sec': tf.header.stamp.sec,
                  'note': 'Connectivity only; freshness and physical calibration are not certified'})
        except TransformException as exc:
            check(parent + '->' + child, False, str(exc))
    for topic in sensor_topics:
        age = time.monotonic() - received[topic] if topic in received else None
        check(topic + ':recent_sample', age is not None and age < 1.0,
              {'receive_age_seconds': age, 'note': 'Receive freshness only; no calibration certification'})
    if args.isolated:
        for topic in ['/cmd_vel', '/chassis_command', '/gimbal_scan_cmd', '/auto_aim_switch']:
            check(topic + ':no_publisher_in_test_domain', not node.get_publishers_info_by_topic(topic),
                  'Isolated diagnostics must not publish hardware commands')
        check('prebuilt_map_localization', False,
              'Diagnostic online map with arbitrary origin; field-map localization not tested')
    unchanged = all((root / name).is_file() and hashlib.sha256((root / name).read_bytes()).hexdigest() == digest
                    for name, digest in hashes.items())
    check('controller_files_unchanged_during_check', unchanged, 'Existing content fingerprints only; not a physical emergency-stop test')
    report = dict(physical_acceptance_passed=False, motion_authorized=False,
                  static_checks_passed=all(c['passed'] for c in checks),
                  ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
                  nodes=sorted(node.get_node_names()), topics=topics,
                  checks=checks, interfaces=interfaces, controller_sha256=hashes,
                  pending=['Physical emergency stop and RC takeover', 'Sensor freshness and localization quality',
                           'Axis calibration and rotating-gimbal navigation', 'Obstacle avoidance and fault stopping'])
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    for item in checks:
        print(('PASS ' if item['passed'] else 'FAIL ') + item['name'])
    print(f'Report: {output}; physical acceptance NOT completed; no motion authorized')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['static_checks_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
