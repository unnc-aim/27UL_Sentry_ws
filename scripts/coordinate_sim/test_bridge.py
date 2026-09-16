"""ROS integration checks in a separate namespace; no Gazebo actuator receives commands."""
import math
import os
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from bridge import Bridge


class BridgeContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '87':
            raise RuntimeError('Use isolated ROS_DOMAIN_ID=87')
        rclpy.init(args=['--ros-args', '-r', '__ns:=/coordinate_bridge_test',
                        '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
        cls.bridge = Bridge()
        cls.driver = Node('probe')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.bridge)
        cls.executor.add_node(cls.driver)
        cls.gt = cls.driver.create_publisher(Odometry, 'ground_truth', 10)
        cls.mode = cls.driver.create_publisher(String, 'coordinate_mode', 10)
        cls.cmd = cls.driver.create_publisher(Twist, 'nav_cmd_vel', 10)
        cls.tf = TransformBroadcaster(cls.driver)
        cls.samples = []
        cls.sub = cls.driver.create_subscription(Twist, 'cmd_vel', lambda m: cls.samples.append(m), 10)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.bridge.destroy_node()
        cls.driver.destroy_node()
        rclpy.shutdown()

    def pump(self, duration, *, transform=True, command=True, heartbeat=True, truth=True):
        self.samples.clear()
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            stamp = self.driver.get_clock().now().to_msg()
            if truth:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.orientation.w = 1.
                self.gt.publish(odom)
            if heartbeat:
                self.mode.publish(String(data='nav'))
            if command:
                vel = Twist()
                vel.linear.x = .2
                self.cmd.publish(vel)
            if transform:
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id = stamp, 'base_footprint'
                tf.child_frame_id = 'gimbal_yaw'
                tf.transform.rotation.z = math.sin(math.pi/4)
                tf.transform.rotation.w = math.cos(math.pi/4)
                self.tf.sendTransform(tf)
            self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(self.samples, 'No actuator output received')

    def assert_stopped(self):
        for msg in self.samples[-3:]:
            self.assertAlmostEqual(msg.linear.x, 0.)
            self.assertAlmostEqual(msg.linear.y, 0.)
            self.assertAlmostEqual(msg.angular.z, 0.)

    def test_frame_transform_and_fault_stops(self):
        self.pump(.8, transform=False)
        self.assert_stopped()
        self.pump(.8)
        self.assertAlmostEqual(self.samples[-1].linear.x, 0., places=5)
        self.assertAlmostEqual(self.samples[-1].linear.y, -.2, places=5)
        for fault in ('command', 'heartbeat', 'transform', 'truth'):
            with self.subTest(fault=fault):
                self.pump(.3)
                self.pump(.8, **{fault: False})
                self.assert_stopped()


if __name__ == '__main__':
    unittest.main()
