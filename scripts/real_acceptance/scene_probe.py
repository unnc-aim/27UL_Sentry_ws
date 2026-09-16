#!/usr/bin/env python3
"""Capture diagnostic laser clearance; no command output or rectangular-axis guess."""
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from clearance import assess, circle_contact


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '88':
        raise RuntimeError('Use diagnostic ROS domain 88')
    rclpy.init()
    node = rclpy.create_node('acceptance_scene_probe', enable_rosout=False, start_parameter_services=False)
    data = []
    node.create_subscription(LaserScan, '/obstacle_scan', lambda m: data.append((time.monotonic(), m)), qos_profile_sensor_data)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    result = {'motion_authorized': False, 'scan_count': len(data),
              'map_direction_verified': False, 'note': 'Ranges are in the scan frame; not chassis clearance or braking certification.'}
    if data:
        history = []
        for received, sample in data:
            valid_ranges = [r for r in sample.ranges if math.isfinite(r) and sample.range_min <= r <= sample.range_max]
            history.append({'finite_returns': len(valid_ranges), 'closest_m': min(valid_ranges) if valid_ranges else None})
        result['scan_history'] = history
        arrival, scan = data[-1]
        valid = [(scan.angle_min+i*scan.angle_increment, r) for i,r in enumerate(scan.ranges)
                 if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        if scan.header.frame_id == 'base_footprint':
            result['directional_geometry'] = assess([[r*math.cos(a),r*math.sin(a)] for a,r in valid])
            window_points = []
            for _, sample in data:
                if sample.header.frame_id != 'base_footprint': continue
                for i, r in enumerate(sample.ranges):
                    if math.isfinite(r) and sample.range_min <= r <= sample.range_max:
                        a = sample.angle_min + i*sample.angle_increment
                        window_points.append([r*math.cos(a),r*math.sin(a)])
            result['directional_geometry']['minimum_observed_contact_over_window_m'] = {
                name:circle_contact(window_points,d) for name,d in
                [('forward',(1,0)),('backward',(-1,0)),('left',(0,1)),('right',(0,-1))]}
        result.update(frame=scan.header.frame_id, receive_age=time.monotonic()-arrival,
                      finite_returns=len(valid), total_beams=len(scan.ranges),
                      closest_return=min((r for _,r in valid),default=None),
                      endpoints=[[r*math.cos(a),r*math.sin(a)] for a,r in valid])
    output = Path('/tmp/sentry-static-acceptance/scene.json')
    output.write_text(json.dumps(result,indent=2)+'\n')
    print({k:v for k,v in result.items() if k not in ('endpoints', 'scan_history')})
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
