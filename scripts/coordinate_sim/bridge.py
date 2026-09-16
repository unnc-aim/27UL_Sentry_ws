"""Simulation odometry and one explicit chassis-to-gimbal velocity adapter.

Only relative ground-truth motion is exposed as odometry. No map pose is seeded.
The existing Gazebo robot_base accepts gimbal-frame Twist, whereas Nav2 here
uses base_footprint. Fresh TF is required; missing TF or commands produce zero.
"""
import copy
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from scenario import rotate, wrap, yaw


class Bridge(Node):
    def __init__(self):
        super().__init__('coordinate_bridge')
        self.nav_spin = self.declare_parameter('nav_spin_speed', 0.0).value
        self.origin = None
        self.gt = None
        self.gt_received = 0.
        self.mode = 'stop'
        self.mode_received = 0.
        self.commands = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf = TransformBroadcaster(self)
        self.vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom = self.create_publisher(Odometry, 'odometry', 10)
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, qos_profile_sensor_data)
        self.create_subscription(String, 'coordinate_mode', self.on_mode, 10)
        for source in ('startup', 'nav'):
            self.create_subscription(Twist, source+'_cmd_vel',
                                     lambda msg, source=source: self.on_cmd(source, msg), 10)
        self.create_timer(.02, self.tick)

    def on_gt(self, msg):
        self.gt, self.gt_received = msg, time.monotonic()

    def on_mode(self, msg):
        self.mode, self.mode_received = msg.data, time.monotonic()

    def on_cmd(self, source, msg):
        self.commands[source] = (msg, time.monotonic())

    def tick(self):
        if self.gt is None or time.monotonic()-self.gt_received > .5:
            self.vel.publish(Twist())
            return
        p, angle = self.gt.pose.pose.position, yaw(self.gt.pose.pose.orientation)
        if self.origin is None:
            self.origin = (p.x, p.y, angle)
        ox, oy, oa = self.origin
        x, y = rotate(p.x-ox, p.y-oy, -oa)
        a = wrap(angle-oa)
        out = copy.deepcopy(self.gt)
        out.header.frame_id, out.child_frame_id = 'odom', 'base_footprint'
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.position.z = 0.
        out.pose.pose.orientation.x = out.pose.pose.orientation.y = 0.
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(a/2), math.cos(a/2)
        self.odom.publish(out)
        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(tf)
        command = Twist()
        msg, received = self.commands.get(self.mode, (None, 0.))
        if msg is not None and time.monotonic()-received < .3 and time.monotonic()-self.mode_received < .5:
            try:
                t = self.buffer.lookup_transform('gimbal_yaw', 'base_footprint', rclpy.time.Time())
                age = (self.get_clock().now()-rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds/1e9
                if not -.1 <= age <= .3:
                    raise ValueError('stale gimbal TF')
                command.linear.x, command.linear.y = rotate(msg.linear.x, msg.linear.y, yaw(t.transform.rotation))
                command.angular.z = msg.angular.z + (self.nav_spin if self.mode == 'nav' else 0.)
            except (TransformException, ValueError):
                pass
        self.vel.publish(command)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '87':
        raise RuntimeError('Simulation bridge requires ROS_DOMAIN_ID=87')
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.vel.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
