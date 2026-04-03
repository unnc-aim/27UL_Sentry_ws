#!/usr/bin/env python3
"""
Standalone NavigateToPose test: gimbal scan, forward 1m, dwell with auto-aim, return.
Publishes auto_aim_switch to gate auto-aim: 0 during scan/nav, 1 at goal.
Publishes gimbal_scan_cmd continuously during scan and navigation phases.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from pb_rm_interfaces.msg import GimbalCmd
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener, TransformException


class SimpleNavTest(Node):
    def __init__(self):
        super().__init__('simple_nav_test')
        self._ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._map_ready = False
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)

        self._gimbal_pub = self.create_publisher(
            GimbalCmd, '/gimbal_scan_cmd',
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))

        self._aim_pub = self.create_publisher(Int32, '/auto_aim_switch', 10)

        self._scanning = False
        self._scan_yaw_speed = 1.0
        self._scan_timer = self.create_wall_timer(1.0 / 20.0, self._scan_timer_cb)

    def _map_cb(self, msg):
        if not self._map_ready:
            self.get_logger().info(
                f'Map received: {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution:.3f} m/cell')
        self._map_ready = True

    def _scan_timer_cb(self):
        if not self._scanning:
            return
        msg = GimbalCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.yaw_type = GimbalCmd.VELOCITY
        msg.pitch_type = GimbalCmd.VELOCITY
        msg.velocity.yaw = self._scan_yaw_speed
        msg.velocity.pitch = 0.0
        msg.velocity.yaw_min_range = -math.pi
        msg.velocity.yaw_max_range = math.pi
        msg.velocity.pitch_min_range = -0.3
        msg.velocity.pitch_max_range = 0.3
        self._gimbal_pub.publish(msg)
        aim = Int32()
        aim.data = 0
        self._aim_pub.publish(aim)

    def set_autoaim(self, enabled):
        msg = Int32()
        msg.data = 1 if enabled else 0
        self._aim_pub.publish(msg)
        state = 'ON' if enabled else 'OFF'
        self.get_logger().info(f'auto_aim_switch = {msg.data} ({state})')

    def start_scan(self, yaw_speed=1.0):
        self._scan_yaw_speed = yaw_speed
        self._scanning = True
        self.set_autoaim(False)

    def stop_scan(self):
        self._scanning = False
        stop = GimbalCmd()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.yaw_type = GimbalCmd.VELOCITY
        stop.pitch_type = GimbalCmd.VELOCITY
        self._gimbal_pub.publish(stop)

    def wait_until_ready(self, timeout=120.0):
        self.get_logger().info('Waiting for /map topic...')
        start = time.time()
        while not self._map_ready:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout:
                self.get_logger().error(f'Map not received after {timeout}s')
                return False
        self.get_logger().info('Map is ready.')

        self.get_logger().info('Waiting for map->base_footprint TF...')
        while True:
            try:
                self._tf_buffer.lookup_transform('map', 'base_footprint',
                                                  rclpy.time.Time())
                break
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.5)
                if time.time() - start > timeout:
                    self.get_logger().error('TF not available')
                    return False
        self.get_logger().info('TF ready.')

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self._ac.wait_for_server(timeout_sec=30.0):
            self.get_logger().fatal('Action server not available')
            return False
        self.get_logger().info('Action server connected.')

        self.get_logger().info('Waiting 5s for costmap to populate...')
        time.sleep(5.0)
        return True

    def get_current_pose(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_footprint',
                                                  rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except TransformException as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

    def gimbal_scan_wait(self, duration):
        """扫描指定秒数（定时器自动发布，这里只等待）"""
        self.get_logger().info(f'Scanning for {duration:.1f}s ...')
        t_start = time.time()
        while (time.time() - t_start) < duration:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f'Scan phase done ({time.time() - t_start:.1f}s)')

    def send_goal_blocking(self, x, y, yaw, timeout=30.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')

        send_future = self._ac.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED')
            return False

        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        t_start = time.time()

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t_start > timeout:
                self.get_logger().warn(f'Navigation timeout ({timeout}s), canceling goal')
                goal_handle.cancel_goal_async()
                t_cancel = time.time()
                while not result_future.done() and (time.time() - t_cancel) < 5.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                return False

        status = result_future.result().status
        names = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }
        self.get_logger().info(f'Result: {names.get(status, status)}')
        return status == GoalStatus.STATUS_SUCCEEDED

    def _feedback_cb(self, feedback_msg):
        pos = feedback_msg.feedback.current_pose.pose.position
        self.get_logger().info(f'  nav pos: ({pos.x:.3f}, {pos.y:.3f})',
                               throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavTest()

    try:
        if not node.wait_until_ready():
            node.get_logger().fatal('System not ready, aborting')
            return

        pose = node.get_current_pose()
        if pose is None:
            node.get_logger().fatal('Cannot get current pose')
            return

        start_x, start_y, start_yaw = pose
        node.get_logger().info(
            f'Start pose: ({start_x:.3f}, {start_y:.3f}, yaw={start_yaw:.3f})')

        fwd_x = start_x + 1.0 * math.cos(start_yaw)
        fwd_y = start_y + 1.0 * math.sin(start_yaw)

        # -- STEP 0: 云台扫描 2 圈，auto-aim OFF --
        node.get_logger().info('===== STEP 0: Gimbal scan (2 rotations), auto-aim OFF =====')
        node.start_scan(yaw_speed=1.0)
        scan_time = 2 * 2.0 * math.pi / 1.0
        node.gimbal_scan_wait(scan_time)

        # -- STEP 1: 前进 1m，扫描继续，auto-aim OFF --
        node.get_logger().info('===== STEP 1: Forward 1m (scan continues) =====')
        ok = node.send_goal_blocking(fwd_x, fwd_y, start_yaw, timeout=30.0)
        if not ok:
            node.get_logger().warn('Forward nav did not succeed, continuing anyway')

        # -- STEP 2: 到达目标，停扫描，auto-aim ON，停留 5s --
        node.get_logger().info('===== STEP 2: At goal, auto-aim ON, dwell 5s =====')
        node.stop_scan()
        node.set_autoaim(True)
        t_dwell = time.time()
        while (time.time() - t_dwell) < 5.0:
            rclpy.spin_once(node, timeout_sec=0.05)

        # -- STEP 3: 返回出发点，恢复扫描，auto-aim OFF --
        node.get_logger().info('===== STEP 3: Return to origin (scan resumes) =====')
        node.start_scan(yaw_speed=1.0)
        ok = node.send_goal_blocking(start_x, start_y, start_yaw, timeout=30.0)
        if not ok:
            node.get_logger().warn('Return nav did not succeed, continuing anyway')

        # -- STEP 4: 回到原点，停扫描，auto-aim ON --
        node.get_logger().info('===== STEP 4: Back at origin, auto-aim ON =====')
        node.stop_scan()
        node.set_autoaim(True)

        node.get_logger().info('===== TEST COMPLETE =====')

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.stop_scan()
        node.set_autoaim(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
