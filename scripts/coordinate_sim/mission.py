"""Automatic global localization, measured rotations and map-landmark regression.

No /initialpose publication and no ground-truth input to localization/readiness.
Ground truth is read only for the independent regression report.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from scenario import clearance, wrap, yaw


class Mission(Node):
    def __init__(self, output, landmarks):
        super().__init__('coordinate_mission')
        self.output = Path(output)
        fixture = yaml.safe_load(Path(landmarks).read_text())
        if fixture['frame'] != 'map':
            raise ValueError('Landmarks must be in map')
        self.goals = fixture['goals']
        self.mode = 'stop'
        self.state = 'waiting'
        self.pose = self.scan = self.map = self.distance = self.gt = self.odom = None
        self.last_yaw = None
        self.rotation = 0.
        self.min_clearance = float('inf')
        self.good_since = None
        self.last_good_scan = -math.inf
        self.match = 0.
        self.startup_started = None
        self.trace = []
        self.trace_time = -math.inf
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.mode_pub = self.create_publisher(String, 'coordinate_mode', 10)
        self.start_pub = self.create_publisher(Twist, 'startup_cmd_vel', 10)
        self.gimbal_pub = self.create_publisher(JointState, 'cmd_gimbal_joint', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, 10)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, 'map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, 'odometry', self.on_odom, 10)
        self.create_subscription(Odometry, 'ground_truth', self.on_truth, qos_profile_sensor_data)
        self.globalize = self.create_client(Empty, 'reinitialize_global_localization')
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(.1, self.heartbeat)
        self.create_timer(1., self.refresh_pose)
        self.create_timer(5., self.status)
        self.report = {'passed': False, 'localization': 'AMCL global, no initialpose',
                       'odometry': 'Gazebo relative motion (ideal)', 'goals': []}

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_pose(self, msg):
        self.pose = msg

    def on_map(self, msg):
        self.map = msg
        a = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        # Small endpoint tolerance mask, no SciPy dependency required.
        occupied = a >= 65
        radius = math.ceil(.2/msg.info.resolution)
        padded = np.pad(occupied, radius)
        near = np.zeros_like(occupied)
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                if math.hypot(dx, dy)*msg.info.resolution <= .2:
                    near |= padded[radius+dy:radius+dy+a.shape[0], radius+dx:radius+dx+a.shape[1]]
        self.distance = np.where(near, 0., 10.)

    def on_odom(self, msg):
        a = yaw(msg.pose.pose.orientation)
        if self.last_yaw is not None:
            self.rotation += wrap(a-self.last_yaw)
        self.last_yaw, self.odom = a, msg

    def on_truth(self, msg):
        self.gt = msg
        p = msg.pose.pose.position
        self.min_clearance = min(self.min_clearance, clearance(p.x, p.y))
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds/1e9
        if stamp-self.trace_time >= .1:
            self.trace_time = stamp
            self.trace.append([stamp, p.x, p.y, yaw(msg.pose.pose.orientation), self.state])

    def heartbeat(self):
        self.mode_pub.publish(String(data=self.mode))
        if self.startup_started is not None:
            t = self.now()-self.startup_started
            # Reproduce two 13s gimbal sweeps separated by 2s; keep scanning during navigation.
            a = .5*min(t, 13.) if t < 15. else 6.5+.5*(t-15.)
            joint = JointState()
            joint.header.stamp = self.get_clock().now().to_msg()
            joint.name, joint.position = ['gimbal_yaw_joint'], [a]
            self.gimbal_pub.publish(joint)

    def refresh_pose(self):
        if self.nomotion.service_is_ready():
            self.nomotion.call_async(Empty.Request())

    def status(self):
        cov = None if self.pose is None else [round(self.pose.pose.covariance[i], 4) for i in (0, 7, 35)]
        self.get_logger().info(f'{self.state}: rotation={self.rotation:.2f}, covariance={cov}, scan_match={self.match:.2f}')

    def localized(self):
        good = False
        if self.pose is not None and self.scan is not None and self.map is not None:
            age = self.now() - rclpy.time.Time.from_msg(self.pose.header.stamp).nanoseconds/1e9
            scan_age = self.now() - rclpy.time.Time.from_msg(self.scan.header.stamp).nanoseconds/1e9
            cov = self.pose.pose.covariance
            try:
                tf = self.buffer.lookup_transform('map', self.scan.header.frame_id,
                                                 rclpy.time.Time.from_msg(self.scan.header.stamp))
                p, a = tf.transform.translation, yaw(tf.transform.rotation)
                ranges = np.asarray(self.scan.ranges)[::4]
                angles = self.scan.angle_min + np.arange(len(self.scan.ranges))[::4]*self.scan.angle_increment + a
                valid = np.isfinite(ranges) & (ranges > self.scan.range_min) & (ranges < self.scan.range_max)
                xs = p.x+ranges[valid]*np.cos(angles[valid])
                ys = p.y+ranges[valid]*np.sin(angles[valid])
                info = self.map.info
                ix = np.floor((xs-info.origin.position.x)/info.resolution).astype(int)
                iy = np.floor((ys-info.origin.position.y)/info.resolution).astype(int)
                inside = (ix >= 0) & (iy >= 0) & (ix < info.width) & (iy < info.height)
                distances = np.full(len(ix), 10.)
                distances[inside] = self.distance[iy[inside], ix[inside]]
                self.match = float(np.mean(distances < .2)) if len(distances) >= 30 else 0.
                good = (-.05 <= age < 3. and -.05 <= scan_age < .5 and
                        cov[0] < .04 and cov[7] < .04 and cov[35] < .025 and self.match > .85)
                if good:
                    self.last_good_scan = self.now()-scan_age
            except TransformException:
                # TF and scan arrive independently. Allow only a bounded transport
                # delay, never substitute an identity transform or an old pose.
                good = self.now()-self.last_good_scan < .3
        if not good:
            self.good_since = None
        elif self.good_since is None:
            self.good_since = self.now()
        return self.good_since is not None and self.now()-self.good_since > 3.

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if predicate():
                return True
        return False

    def run(self):
        if not self.spin_until(lambda: self.scan is not None and self.odom is not None and
                               self.map is not None and self.globalize.service_is_ready(), 90):
            raise RuntimeError('Simulation sensors/map/global localization service not ready')
        future = self.globalize.call_async(Empty.Request())
        if not self.spin_until(future.done, 10):
            raise RuntimeError('Global localization request timed out')
        future.result()
        self.state, self.mode = 'startup_and_global_localization', 'startup'
        self.startup_started = self.now()
        rotation_start = self.rotation
        deadline = time.monotonic()+240
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            cmd = Twist()
            rotated = self.rotation-rotation_start >= 4*math.pi
            cmd.angular.z = 0. if rotated else .6
            self.start_pub.publish(cmd)
            ready = self.localized()
            if rotated and self.now()-self.startup_started >= 28. and ready:
                break
        else:
            raise RuntimeError('Automatic localization/startup did not converge; navigation inhibited')
        self.mode = 'stop'
        self.report['startup_rotation_rad'] = self.rotation-rotation_start
        self.report['localization_error'] = self.truth_error()
        if not self.spin_until(lambda: self.nav.server_is_ready(), 30):
            raise RuntimeError('Nav2 action server not ready')
        for name in ('east', 'north', 'home'):
            self.state = 'navigate_'+name
            x, y, a = self.goals[name]
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(a/2), math.cos(a/2)
            self.mode = 'nav'
            future = self.nav.send_goal_async(goal)
            if not self.spin_until(future.done, 10):
                raise RuntimeError('Goal response timed out')
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Goal rejected: '+name)
            result = handle.get_result_async()
            deadline = time.monotonic()+180
            while not result.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
                if not self.localized():
                    self.mode = 'stop'
                    handle.cancel_goal_async()
                    raise RuntimeError('Localization confidence lost; motion inhibited')
            self.mode = 'stop'
            if not result.done():
                handle.cancel_goal_async()
                raise RuntimeError('Navigation timed out: '+name)
            if result.result().status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError('Nav2 failed: '+name)
            p = self.gt.pose.pose.position
            error = math.hypot(p.x-x, p.y-y)
            record = {'name': name, 'target': [x, y], 'actual': [p.x, p.y], 'error_m': error,
                      'localization_error': self.truth_error()}
            self.report['goals'].append(record)
            self.get_logger().info(json.dumps(record))
            if error > .35:
                raise RuntimeError('Independent ground-truth arrival check failed')
        self.report['passed'] = self.min_clearance > .32
        if not self.report['passed']:
            raise RuntimeError('Obstacle clearance below test threshold')

    def truth_error(self):
        tf = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        p, q = self.gt.pose.pose.position, self.gt.pose.pose.orientation
        return {'position_m': math.hypot(tf.transform.translation.x-p.x, tf.transform.translation.y-p.y),
                'yaw_rad': abs(wrap(yaw(tf.transform.rotation)-yaw(q)))}

    def finish(self):
        self.mode = 'stop'
        self.report['minimum_center_clearance_m'] = self.min_clearance if math.isfinite(self.min_clearance) else None
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(self.report, indent=2)+'\n')
        with self.output.with_suffix('.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_time', 'world_x', 'world_y', 'world_yaw', 'state'])
            writer.writerows(self.trace)
        self.get_logger().info(f'Result: {self.output}, passed={self.report["passed"]}')
        for _ in range(5):
            self.mode_pub.publish(String(data='stop'))
            self.start_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=.05)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '87':
        raise RuntimeError('Simulation regression requires ROS_DOMAIN_ID=87')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--landmarks', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Mission(args.output, args.landmarks)
    try:
        node.run()
    except (Exception, KeyboardInterrupt) as exc:
        node.report['error'] = str(exc)
        node.report['passed'] = False
        node.get_logger().error(str(exc))
    finally:
        if rclpy.ok():
            node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if node.report['passed'] else 1)


if __name__ == '__main__':
    main()
