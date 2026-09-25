#!/usr/bin/env python3
"""Check velocity frames, fresh odometry, stopping and native recovery in a private ROS domain."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from example_interfaces.msg import Float32
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist, TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import BackUp
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = Path(os.environ.get('SENTRY_TEST_OUTPUT', ROOT/'scripts/real_acceptance/results/control_review_20260925'))
OUTPUT.mkdir(exist_ok=True, parents=True)


class Check(Node):
    def __init__(self):
        super().__init__('velocity_frame_check')
        self.angle = 0.
        self.global_angle = 0.
        self.send_odom = False
        self.send_cmd = True
        self.quaternion_valid = True
        self.velocity = (.3, .1, 0.)
        self.outputs = []
        self.transforms = []
        self.obstacle = False
        self.odom_pub = self.create_publisher(Odometry, 'test_odometry', 10)
        self.cmd_pub = self.create_publisher(Twist, 'test_velocity_input', 10)
        self.plan_pub = self.create_publisher(NavPath, 'local_plan', 10)
        self.spin_pub = self.create_publisher(Float32, 'test_spin', 10)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TwistStamped, 'test_velocity_output', self.outputs.append, 20)
        self.create_subscription(TFMessage, 'tf', lambda m: self.transforms.extend(
            tf for tf in m.transforms if tf.child_frame_id == 'test_fake'), 20)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_pub = self.create_publisher(Costmap, 'test_costmap', qos)
        self.footprint_pub = self.create_publisher(PolygonStamped, 'test_footprint', qos)
        self.create_timer(.05, self.publish)

    def publish(self):
        stamp = self.get_clock().now()
        if self.send_odom:
            m = Odometry()
            m.header.frame_id, m.child_frame_id = 'odom', 'test_gimbal'
            m.header.stamp = stamp.to_msg()
            if self.quaternion_valid:
                m.pose.pose.orientation.z = math.sin(self.angle/2)
                m.pose.pose.orientation.w = math.cos(self.angle/2)
            else:
                m.pose.pose.orientation.w = 0.
            self.odom_pub.publish(m)
            tf = TransformStamped()
            tf.header = m.header
            tf.child_frame_id = 'test_gimbal'
            tf.transform.rotation = m.pose.pose.orientation
            map_tf = TransformStamped()
            map_tf.header.frame_id, map_tf.child_frame_id = 'map', 'odom'
            map_tf.header.stamp = stamp.to_msg()
            map_tf.transform.rotation.z = math.sin(self.global_angle/2)
            map_tf.transform.rotation.w = math.cos(self.global_angle/2)
            self.broadcaster.sendTransform([map_tf, tf])
        if self.send_cmd:
            v = Twist()
            v.linear.x, v.linear.y, v.angular.z = self.velocity
            self.cmd_pub.publish(v)
        path = NavPath()
        path.header.frame_id = 'test_fake'
        path.header.stamp = (stamp-Duration(seconds=2.)).to_msg()
        self.plan_pub.publish(path)
        costmap = Costmap()
        costmap.header.frame_id = 'odom'
        costmap.header.stamp = stamp.to_msg()
        costmap.metadata.resolution = .1
        costmap.metadata.size_x = costmap.metadata.size_y = 100
        costmap.metadata.origin.position.x = costmap.metadata.origin.position.y = -5.
        costmap.metadata.origin.orientation.w = 1.
        costmap.data = [0]*10000
        if self.obstacle:
            for row in range(45, 55):
                for col in range(44, 48):
                    costmap.data[row*100+col] = 254
        self.costmap_pub.publish(costmap)
        footprint = PolygonStamped()
        footprint.header = costmap.header
        footprint.polygon.points = [Point32(x=x,y=y,z=0.) for x,y in ((-.15,-.15),(.15,-.15),(.15,.15),(-.15,.15))]
        self.footprint_pub.publish(footprint)

    def spin(self, seconds, predicate=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate is not None and predicate():
                return True
        return predicate is None

    def response(self, future, seconds=5):
        assert self.spin(seconds, future.done), 'ROS response timed out'
        return future.result()

    def collect(self, seconds=.8):
        self.spin(.2)
        self.outputs.clear()
        self.spin(seconds)
        assert len(self.outputs) >= 8, len(self.outputs)
        return self.outputs


def zero(messages):
    return all(abs(m.twist.linear.x)+abs(m.twist.linear.y)+abs(m.twist.angular.z)<1e-9 for m in messages)


def check_odometry(n, start):
    poses, clouds = [], []
    n.create_subscription(Odometry, 'generated_odometry', poses.append, 20)
    n.create_subscription(PointCloud2, 'converted_scan', clouds.append, 10)
    source = n.create_publisher(Odometry, 'scan_test_lidar_odom', 10)
    scans = n.create_publisher(PointCloud2, 'scan_test_points', 10)
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    start('scan_generation', ['ros2', 'run', 'sensor_scan_generation', 'sensor_scan_generation_node',
          '--ros-args', '-r', '__node:=scan_frame_check', '-p', 'lidar_frame:=scan_test_lidar',
          '-p', 'base_frame:=scan_test_base', '-p', 'robot_base_frame:=scan_test_gimbal',
          '-r', 'lidar_odometry:=scan_test_lidar_odom', '-r', 'registered_scan:=scan_test_points',
          '-r', 'odometry:=generated_odometry', '-r', 'sensor_scan:=converted_scan'])
    assert n.spin(5, lambda: source.get_subscription_count() > 0)

    def message(stamp, x):
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'scan_test_lidar'
        msg.header.stamp = stamp.to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.z = msg.pose.pose.orientation.w = math.sqrt(.5)
        return msg

    def publish_frame(msg):
        source.publish(msg)
        scans.publish(create_cloud(Header(stamp=msg.header.stamp, frame_id='odom'), fields,
                                   [(msg.pose.pose.position.x, 1., 0., 1.)]))

    publish_frame(message(n.get_clock().now(), 10.))
    n.spin(.6)
    assert poses == []
    static = StaticTransformBroadcaster(n)
    base_to_gimbal = TransformStamped()
    base_to_gimbal.header.stamp = n.get_clock().now().to_msg()
    base_to_gimbal.header.frame_id, base_to_gimbal.child_frame_id = 'scan_test_base', 'scan_test_gimbal'
    base_to_gimbal.transform.rotation.z = base_to_gimbal.transform.rotation.w = math.sqrt(.5)
    gimbal_to_lidar = TransformStamped()
    gimbal_to_lidar.header = Header(stamp=base_to_gimbal.header.stamp, frame_id='scan_test_gimbal')
    gimbal_to_lidar.child_frame_id = 'scan_test_lidar'
    gimbal_to_lidar.transform.rotation.w = 1.
    static.sendTransform([base_to_gimbal, gimbal_to_lidar])
    n.spin(.4)
    stamp = n.get_clock().now()-Duration(seconds=.15)
    for i in range(3):
        publish_frame(message(stamp+Duration(seconds=.05*i), 10.+.05*i))
        n.spin(.03)
    assert n.spin(2, lambda: len(poses) >= 3)
    first = poses[0].twist.twist
    assert abs(first.linear.x)+abs(first.linear.y)+abs(first.angular.z) < 1e-9
    for msg in poses[1:3]:
        assert abs(msg.twist.twist.linear.x) < 1e-6
        assert abs(msg.twist.twist.linear.y+1.) < 1e-6
        assert msg.child_frame_id == 'scan_test_gimbal'
    poses.clear()
    for _ in range(20):
        last = message(n.get_clock().now(), 10.1)
        publish_frame(last)
        n.spin(.05)
    assert len(poses) >= 15, len(poses)
    assert n.spin(3, lambda: len(clouds) > 0)
    points = list(read_points(clouds[-1], field_names=('x', 'y', 'z')))
    assert clouds[-1].header.frame_id == 'scan_test_lidar'
    assert abs(points[0][0]-1.) < 1e-5 and abs(points[0][1]) < 1e-5
    print('PASS: frame-aligned odometry is continuous; first velocity is zero; measurement time and child frame are respected', flush=True)


def main():
    children, logs = [], []
    def start(name, command):
        stream = (OUTPUT/(name+'.log')).open('w')
        logs.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(process)
        return process
    rclpy.init()
    n = Check()
    try:
        start('adapter', ['ros2','run','fake_vel_transform','fake_vel_transform_node','--ros-args',
            '-p','robot_base_frame:=test_gimbal','-p','fake_robot_base_frame:=test_fake',
            '-p','odom_topic:=test_odometry','-p','input_cmd_vel_topic:=test_velocity_input',
            '-p','output_cmd_vel_topic:=test_velocity_output','-p','cmd_spin_topic:=test_spin',
            '-p','translation_scale:=-1.0','-p','init_spin_speed:=0.0'])
        n.spin(2)
        assert zero(n.collect()), 'Expected zero output while odometry starts'
        n.send_odom = True
        for angle in (0., math.pi/2, math.pi, -math.pi/2, .4):
            n.angle = angle
            expected = (-(.3*math.cos(angle)+.1*math.sin(angle)),
                        -(-.3*math.sin(angle)+.1*math.cos(angle)))
            for m in n.collect():
                assert abs(m.twist.linear.x-expected[0]) < 1e-6
                assert abs(m.twist.linear.y-expected[1]) < 1e-6
                assert m.header.frame_id == 'test_gimbal'
        print('PASS: every command uses current yaw, including 90 and 180 degrees and old local-plan stamps', flush=True)
        n.spin_pub.publish(Float32(data=1.7))
        assert all(abs(m.twist.angular.z-1.7) < 1e-6 for m in n.collect())
        n.spin_pub.publish(Float32(data=0.))
        assert all(abs(m.twist.angular.z) < 1e-9 for m in n.collect())
        print('PASS: explicit cmd_spin controls spin during translation', flush=True)
        n.velocity = (0.,0.,0.)
        assert zero(n.collect())
        n.send_cmd = False
        n.outputs.clear()
        n.spin(.8)
        assert zero(n.outputs), 'An earlier movement command was repeated after stopping'
        n.send_cmd = True
        n.velocity = (.3,.1,0.)
        n.send_odom = False
        n.spin(.7)
        assert zero(n.collect())
        n.send_odom = True
        n.quaternion_valid = False
        assert zero(n.collect())
        n.quaternion_valid = True
        n.velocity = (float('nan'),0.,0.)
        assert zero(n.collect())
        print('PASS: zero commands, stale odometry and invalid input produce zero movement', flush=True)

        check_odometry(n, start)

        params = yaml.safe_load((ROOT/'src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml').read_text())
        assert params['point_lio']['ros__parameters']['odometry']['publish_odometry_without_downsample'] is False
        profile = dict(params['behavior_server']['ros__parameters'])
        assert profile['backup']['plugin'] == 'nav2_behaviors/BackUp'
        assert profile['global_frame'] == 'odom'
        profile.update(behavior_plugins=['backup'], robot_base_frame='test_fake',
                       costmap_topic='test_costmap', footprint_topic='test_footprint')
        config = OUTPUT/'behavior_test.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': profile}}))
        n.send_cmd = False
        n.angle = 0.
        start('native_backup', ['ros2','run','nav2_behaviors','behavior_server','--ros-args',
              '-r','__node:=frame_test_behaviors','-r','cmd_vel:=test_velocity_input','--params-file',str(config)])
        state = n.create_client(ChangeState, '/frame_test_behaviors/change_state')
        assert n.spin(8, state.service_is_ready)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            assert n.response(state.call_async(request)).success
        action = ActionClient(n, BackUp, 'backup')
        assert n.spin(5, action.server_is_ready)
        for angle, global_angle in ((0.,0.),(math.pi/2,math.pi/2),(math.pi,-math.pi),(-math.pi/2,.7)):
            n.angle, n.global_angle = angle, global_angle
            n.spin(.4)
            n.outputs.clear()
            goal = BackUp.Goal()
            goal.target.x = -1.
            goal.speed = .2
            goal.time_allowance.sec = 5
            handle = n.response(action.send_goal_async(goal))
            assert handle.accepted
            assert n.spin(3, lambda: len([m for m in n.outputs if abs(m.twist.linear.x)+abs(m.twist.linear.y)>.05]) >= 3), OUTPUT
            expected = (.2*math.cos(angle), -.2*math.sin(angle))
            moving = [m for m in n.outputs if abs(m.twist.linear.x)+abs(m.twist.linear.y)>.05]
            for m in moving:
                assert abs(m.twist.linear.x-expected[0]) < 1e-5, (angle,m.twist)
                assert abs(m.twist.linear.y-expected[1]) < 1e-5, (angle,m.twist)
            n.response(handle.cancel_goal_async())
            n.response(handle.get_result_async())
            n.spin(.3)
            assert zero(n.outputs[-1:])
        print('PASS: native BackUp uses odom consistently across map rotations and chassis headings', flush=True)
        n.obstacle = True
        n.angle = 0.
        n.spin(.5)
        n.outputs.clear()
        handle = n.response(action.send_goal_async(goal))
        assert handle.accepted
        result = n.response(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED
        n.spin(.2)
        assert n.outputs and zero(n.outputs)
        print('PASS: native recovery stops for an obstacle in the commanded direction', flush=True)
        assert params['pb_teleop_twist_joy_node']['ros__parameters']['publish_stamped_twist'] is True
        start('teleop_type', ['ros2', 'run', 'pb_teleop_twist_joy', 'pb_teleop_twist_joy_node',
              '--ros-args', '-r', '__node:=pb_teleop_twist_joy_node',
              '-r', 'cmd_vel:=test_velocity_output', '--params-file',
              str(ROOT/'src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml')])
        assert n.spin(5, lambda: n.count_publishers('test_velocity_output') == 2)
        types = dict(n.get_topic_names_and_types())['/test_velocity_output']
        assert types == ['geometry_msgs/msg/TwistStamped'], types
        print('PASS: adapter and teleop share one velocity message type', flush=True)
        assert all(child.poll() is None for child in children)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
        n.destroy_node()
        rclpy.try_shutdown()
        for stream in logs:stream.close()


if __name__ == '__main__':
    main()
