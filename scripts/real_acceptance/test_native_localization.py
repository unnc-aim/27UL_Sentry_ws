#!/usr/bin/env python3
"""Exercise startup ordering and both native AMCL launch forms in an isolated ROS domain."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('SENTRY_TEST_DOMAIN_ID', '177')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
assert 100 <= int(os.environ['ROS_DOMAIN_ID']) <= 230

import rclpy
from btcpp_ros2_interfaces.action import ExecuteTree
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = Path(tempfile.mkdtemp(prefix='sentry-amcl-check-'))
CHILDREN = []
LOGS = []


def start(name, command):
    stream = (OUTPUT/(name+'.log')).open('w')
    LOGS.append(stream)
    child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    CHILDREN.append(child)
    return child


def stop(child):
    if child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=5)
    CHILDREN.remove(child)


def client(name, target='rmul_2025_reality', timeout=45.):
    return start(name, ['ros2', 'run', 'pb2025_sentry_behavior', 'pb2025_sentry_behavior_client',
                       '--ros-args', '-r', '__ns:=/'+name, '-r', '/tf:=tf', '-r', '/tf_static:=tf_static',
                       '-p', 'target_tree:='+target, '-p', 'wait_for_navigation:=true',
                       '-p', 'navigation_startup_timeout:='+str(timeout)])


def spin(executor, seconds, predicate=None):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.02)
        if predicate is not None and predicate():
            return True
    return predicate is None


class Inputs(Node):
    def __init__(self, namespace, native=False):
        super().__init__('synthetic_inputs', namespace=namespace,
                         cli_args=['--ros-args', '-r', '/tf:=/'+namespace+'/tf'])
        self.native = native
        self.goals, self.poses, self.scans, self.map_transforms = [], [], [], []
        self.map_enabled = self.tf_enabled = self.active = False
        self.release_global = False
        self.global_calls = self.nomotion_calls = 0
        self.tf_age = self.scan_age = 0.
        self.pose_stamp = None
        self.variance = .01
        self.frame = 'map'
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.poses.append, 10)
        self.create_subscription(LaserScan, 'obstacle_scan', self.scans.append, qos_profile_sensor_data)
        self.create_subscription(TFMessage, 'tf', lambda msg: self.map_transforms.extend(
            tf for tf in msg.transforms if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom'), 10)
        self.tree_server = ActionServer(self, ExecuteTree, 'pb2025_sentry_behavior', self.execute_tree)
        self.nav_server = ActionServer(self, NavigateThroughPoses, 'navigate_through_poses', self.execute_nav)
        if native:
            self.cloud_pub = self.create_publisher(PointCloud2, 'terrain_map_ext', 10)
            self.cloud_points = self.synthetic_points()
        else:
            self.map_pub = self.create_publisher(OccupancyGrid, 'map',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.scan_pub = self.create_publisher(LaserScan, 'obstacle_scan', 10)
            self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)
            self.create_service(GetState, 'amcl/get_state', self.get_state)
            self.create_service(Empty, 'reinitialize_global_localization', self.globalize,
                                callback_group=ReentrantCallbackGroup())
            self.create_service(Empty, 'request_nomotion_update', self.nomotion)
        self.create_timer(.05, self.publish)

    def execute_tree(self, goal):
        self.goals.append(goal.request.target_tree)
        goal.succeed()
        result = ExecuteTree.Result()
        result.return_message = 'Synthetic action completed'
        return result

    def execute_nav(self, goal):
        goal.succeed()
        return NavigateThroughPoses.Result()

    def get_state(self, request, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE if self.active else State.PRIMARY_STATE_INACTIVE
        return response

    def globalize(self, request, response):
        self.global_calls += 1
        until = time.monotonic()+10
        while not self.release_global and time.monotonic() < until:
            time.sleep(.01)
        return response

    def nomotion(self, request, response):
        self.nomotion_calls += 1
        return response

    @staticmethod
    def synthetic_points():
        sys.path.insert(0, str(ROOT/'scripts/coordinate_sim'))
        from scenario import BOXES
        points = []
        x, y, yaw = 1., 1., .4
        for i in range(720):
            angle = -math.pi+i*math.pi/360
            dx, dy = math.cos(angle+yaw), math.sin(angle+yaw)
            distance = math.inf
            for bx, by, sx, sy in BOXES:
                lo, hi = 0., 10.
                for origin, direction, lower, upper in ((x, dx, bx-sx/2, bx+sx/2),
                                                          (y, dy, by-sy/2, by+sy/2)):
                    if abs(direction) < 1e-9:
                        if origin < lower or origin > upper:
                            hi = -1.
                    else:
                        a, b = (lower-origin)/direction, (upper-origin)/direction
                        lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
                if 0 < lo <= hi:
                    distance = min(distance, lo)
            if math.isfinite(distance):
                points.append((distance*math.cos(angle), distance*math.sin(angle), .5, 1.))
        return points

    def publish(self):
        stamp = self.get_clock().now()
        if self.tf_enabled:
            transforms = []
            for parent, child in [('odom', 'base_footprint'), ('base_footprint', 'gimbal_yaw')]:
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = parent, child
                tf.header.stamp = (stamp-Duration(seconds=self.tf_age)).to_msg()
                tf.transform.rotation.w = 1.
                transforms.append(tf)
            if not self.native:
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
                tf.header.stamp = stamp.to_msg()
                tf.transform.rotation.w = 1.
                transforms.append(tf)
            self.broadcaster.sendTransform(transforms)
        if self.native:
            fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
            self.cloud_pub.publish(create_cloud(Header(stamp=stamp.to_msg(), frame_id='odom'), fields,
                                                self.cloud_points))
            return
        if self.map_enabled:
            m = OccupancyGrid()
            m.header.frame_id, m.header.stamp = 'map', stamp.to_msg()
            m.info.width = m.info.height = 20
            m.info.resolution = .1
            m.info.origin.orientation.w = 1.
            m.data = [0]*400
            self.map_pub.publish(m)
        scan = LaserScan()
        scan.header.frame_id = 'base_footprint'
        scan.header.stamp = (stamp-Duration(seconds=self.scan_age)).to_msg()
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/45
        scan.range_min, scan.range_max, scan.ranges = .3, 10., [2.]*90
        self.scan_pub.publish(scan)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.pose_stamp if self.pose_stamp is not None else stamp.to_msg()
        pose.pose.pose.orientation.w = 1.
        for index in (0, 7, 35):
            pose.pose.covariance[index] = self.variance
        self.pose_pub.publish(pose)


def check_client(executor):
    n = Inputs('startup_test')
    executor.add_node(n)
    process = client('startup_test')
    try:
        spin(executor, 2)
        assert n.global_calls == 0 and n.goals == []
        n.map_enabled = True
        spin(executor, .8)
        assert n.global_calls == 0 and n.goals == []
        n.tf_enabled = True
        spin(executor, .8)
        assert n.global_calls == 0 and n.goals == []
        n.active = True
        assert spin(executor, 3, lambda: n.global_calls == 1)
        spin(executor, 1)
        assert n.goals == []
        n.pose_stamp = n.get_clock().now().to_msg()
        n.release_global = True
        spin(executor, 1.5)
        assert n.goals == []
        for name, value in [('variance', .5), ('variance', float('nan')),
                            ('frame', 'odom'), ('scan_age', 2.), ('tf_age', 2.)]:
            n.pose_stamp = None
            n.variance, n.frame, n.scan_age, n.tf_age = .01, 'map', 0., 0.
            setattr(n, name, value)
            spin(executor, 1.)
            assert n.goals == [], name
        n.tf_age = 0.
        assert spin(executor, 9, lambda: len(n.goals) == 1)
        spin(executor, 1)
        assert n.goals == ['rmul_2025_reality'] and n.global_calls == 1 and n.nomotion_calls >= 2
        assert process.poll() is None
        print('PASS: map, TF, activation, response ordering, fresh pose, covariance and one execution')
    finally:
        n.release_global = True
        stop(process)
        executor.remove_node(n)
        n.destroy_node()
    for namespace, tree, timeout, expected in (
        ('other_tree_test', 'test_attacked_feedback', 2., ['test_attacked_feedback']),
        ('timeout_test', 'rmul_2025_reality', 1., []),
    ):
        n = Inputs(namespace)
        executor.add_node(n)
        process = client(namespace, tree, timeout)
        spin(executor, 3)
        assert n.goals == expected and n.global_calls == 0
        if namespace == 'timeout_test':
            assert 'startup stopped after timeout' in (OUTPUT/(namespace+'.log')).read_text()
        stop(process)
        executor.remove_node(n)
        n.destroy_node()
    print('PASS: other trees retain their startup behavior; timeout ends the startup attempt')


def check_probe(executor):
    namespace = 'probe_test'
    node = Inputs(namespace)
    node.map_enabled = node.tf_enabled = node.active = node.release_global = True
    executor.add_node(node)
    output = OUTPUT/'probe.json'
    process = start(namespace, [sys.executable, str(ROOT/'scripts/real_acceptance/localization_probe.py'),
                    '--globalize', '--seconds', '6', '--output', str(output),
                    '--ros-args', '-r', '__ns:=/'+namespace, '-r', '/tf:=tf'])
    seen_ready = False
    try:
        deadline = time.monotonic()+15
        while process.poll() is None and time.monotonic() < deadline:
            spin(executor, .1)
            if output.exists():
                seen_ready |= json.loads(output.read_text())['latest']['candidate_stable']
        assert process.poll() == 0 and seen_ready, OUTPUT
        result = json.loads(output.read_text())
        assert result['globalization_requested'] and not result['latest']['candidate_stable']
        assert node.global_calls == 1 and node.nomotion_calls >= 2
        print('PASS: diagnostic initialization, stable pose reporting and final false status')
    finally:
        stop(process)
        executor.remove_node(node)
        node.destroy_node()


def check_native(executor, composition):
    namespace = 'native_components' if composition else 'native_plain'
    sys.path.insert(0, str(ROOT/'scripts/coordinate_sim'))
    from scenario import make_arena
    make_arena(OUTPUT/'arena')
    launch_path = ROOT/'src/pb2025_sentry_nav/pb2025_nav_bringup/launch/localization_launch.py'
    wrapper = OUTPUT/(namespace+'.launch.py')
    configs = dict(namespace=namespace, use_amcl='True', use_sim_time='False', autostart='True',
                   use_respawn='False', log_level='info', prior_pcd_file='',
                   map=str(OUTPUT/'arena/map.yaml'), use_composition=str(composition),
                   container_name='nav2_container', params_file=str(ROOT/'src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml'))
    wrapper.write_text('''import runpy
from launch import LaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace, SetRemap

def start(context):
    context.launch_configurations.update('''+repr(configs)+''')
    actions = runpy.run_path('''+repr(str(launch_path))+''')['launch_localization'](context)
    # Synthetic odometry replaces the Point-LIO process for this test.
    actions = actions[1:]
    if '''+repr(composition)+''':
        actions.insert(0, Node(package='rclcpp_components', executable='component_container_isolated',
                               name='nav2_container', output='screen'))
    return [GroupAction([PushRosNamespace('''+repr(namespace)+'''),
                         SetRemap('/tf', 'tf'), SetRemap('/tf_static', 'tf_static'), *actions])]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=start)])
''')
    n = Inputs(namespace, native=True)
    n.tf_enabled = True
    executor.add_node(n)
    launch = start(namespace+'_launch', ['ros2', 'launch', str(wrapper)])
    behavior = client(namespace)
    try:
        assert spin(executor, 35, lambda: len(n.poses) >= 4), (namespace, OUTPUT)
        assert len(n.scans) >= 4 and n.map_transforms
        stamps = {(p.header.stamp.sec, p.header.stamp.nanosec) for p in n.poses}
        assert len(stamps) >= 4
        assert all(p.header.frame_id == 'map' for p in n.poses)
        assert all(s.header.frame_id == 'base_footprint' for s in n.scans)
        assert n.count_publishers('initialpose') == 0
        names = [name for name, ns in n.get_node_names_and_namespaces() if ns == '/'+namespace]
        assert 'amcl' in names and 'map_server' in names and 'small_gicp_relocalization' not in names
        assert launch.poll() is None and behavior.poll() is None
        text = (OUTPUT/(namespace+'.log')).read_text()
        assert text.count('AMCL global initialization completed') == 1
        print('PASS:', namespace, 'native AMCL, projected scans, stationary pose updates, TF, automatic initialization')
    finally:
        stop(behavior)
        stop(launch)
        executor.remove_node(n)
        n.destroy_node()


def main():
    print('Test output:', OUTPUT, flush=True)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        check_client(executor)
        check_probe(executor)
        check_native(executor, False)
        check_native(executor, True)
    finally:
        for child in list(CHILDREN):
            stop(child)
        executor.shutdown()
        rclpy.try_shutdown()
        for stream in LOGS:
            stream.close()


if __name__ == '__main__':
    main()
