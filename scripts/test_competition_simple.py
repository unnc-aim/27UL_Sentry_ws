#!/usr/bin/env python3
"""
简化版正赛控制器测试 — 验证状态机核心逻辑

与正赛控制器差异:
  - 无裁判系统, 无自瞄, 不开拨弹盘
  - 航点为相对坐标: 起点 → (起点 + 1m) → 起点  (往返)
  - 手动启动, 不依赖 systemd
  - 2 轮往返后自动结束

关键: 使用 NavigateToPose (非 ThroughPoses), 与 test_simple_nav.py 一致

Usage:
    python3 ~/sentry_ws/scripts/test_competition_simple.py
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from pb_rm_interfaces.msg import GimbalCmd
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener

SCAN_SPEED = 0.5
DWELL_SECS = 5.0
NAV_TIMEOUT = 30.0
READY_TIMEOUT = 120.0
NUM_ROUNDS = 2
FORWARD_DISTANCE = 1.0


class TestCompetitionSimple(Node):
    def __init__(self):
        super().__init__('test_competition_simple')

        self._nav_ac = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._tf_buf = Buffer()
        self._tf_lis = TransformListener(self._tf_buf, self)

        self._map_ok = False
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map,
            QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))

        qos_be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._gimbal_pub = self.create_publisher(GimbalCmd, '/gimbal_scan_cmd', qos_be)
        self._aim_pub = self.create_publisher(Int32, '/auto_aim_switch', 10)
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._scanning = False
        self._scan_speed = SCAN_SPEED
        self._scan_timer = self.create_timer(1.0 / 20.0, self._scan_tick)

    def _on_map(self, msg):
        if not self._map_ok:
            self.get_logger().info(
                f'Map: {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution:.3f} m/cell')
        self._map_ok = True

    def _scan_tick(self):
        if not self._scanning:
            return
        g = GimbalCmd()
        g.header.stamp = self.get_clock().now().to_msg()
        g.yaw_type = GimbalCmd.VELOCITY
        g.pitch_type = GimbalCmd.VELOCITY
        g.velocity.yaw = self._scan_speed
        g.velocity.pitch = 0.0
        g.velocity.yaw_min_range = -math.pi
        g.velocity.yaw_max_range = math.pi
        g.velocity.pitch_min_range = -0.3
        g.velocity.pitch_max_range = 0.3
        self._gimbal_pub.publish(g)

    # ── helpers ──────────────────────────────────────────────────

    def _spin_ros(self, secs):
        t0 = time.time()
        while (time.time() - t0) < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _stop_chassis(self):
        t = Twist()
        self._vel_pub.publish(t)

    def _set_aim(self, on):
        m = Int32(); m.data = 1 if on else 0
        self._aim_pub.publish(m)

    def _scan_on(self):
        self._scanning = True

    def _scan_off(self):
        self._scanning = False
        g = GimbalCmd()
        g.header.stamp = self.get_clock().now().to_msg()
        g.yaw_type = GimbalCmd.VELOCITY
        g.pitch_type = GimbalCmd.VELOCITY
        self._gimbal_pub.publish(g)

    def get_current_pose(self):
        try:
            t = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except TransformException as e:
            self.get_logger().error(f'TF lookup: {e}')
            return None

    # ── wait ─────────────────────────────────────────────────────

    def wait_until_ready(self):
        log = self.get_logger()
        t0 = time.time()

        log.info('等待 /map ...')
        while not self._map_ok:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - t0 > READY_TIMEOUT:
                log.error('Map 超时'); return False
        log.info('Map OK')

        log.info('等待 TF (map → base_footprint) ...')
        while True:
            try:
                self._tf_buf.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time())
                break
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.5)
                if time.time() - t0 > READY_TIMEOUT:
                    log.error('TF 超时'); return False
        log.info('TF OK')

        log.info('等待 navigate_to_pose action ...')
        if not self._nav_ac.wait_for_server(timeout_sec=30.0):
            log.fatal('Action server 不可用'); return False
        log.info('Action server OK')

        log.info('costmap 填充 (5s) ...')
        self._spin_ros(5.0)
        return True

    # ── navigate (NavigateToPose — 与 test_simple_nav.py 一致) ──

    def navigate_to(self, x, y, yaw, timeout=NAV_TIMEOUT):
        log = self.get_logger()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        log.info(f'导航 → ({x:.3f}, {y:.3f}, yaw={yaw:.3f})')

        fut = self._nav_ac.send_goal_async(goal, feedback_callback=self._fb)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        gh = fut.result()
        if not gh.accepted:
            log.error('目标被拒绝'); return False

        log.info('目标已接受')
        fut_res = gh.get_result_async()
        t0 = time.time()
        while not fut_res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout:
                log.warn(f'超时 ({timeout}s), 取消')
                gh.cancel_goal_async()
                tc = time.time()
                while not fut_res.done() and (time.time() - tc) < 5.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                self._stop_chassis()
                return False

        self._stop_chassis()

        status = fut_res.result().status
        names = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }
        log.info(f'结果: {names.get(status, str(status))}')
        return status == GoalStatus.STATUS_SUCCEEDED

    def _fb(self, fb_msg):
        pos = fb_msg.feedback.current_pose.pose.position
        self.get_logger().info(
            f'  pos: ({pos.x:.3f}, {pos.y:.3f})',
            throttle_duration_sec=2.0)

    # ── main ─────────────────────────────────────────────────────

    def run(self):
        log = self.get_logger()

        log.info('===== 简化正赛测试 =====')
        log.info(f'往返距离: {FORWARD_DISTANCE}m, 轮次: {NUM_ROUNDS}')

        if not self.wait_until_ready():
            log.fatal('系统未就绪'); return

        pose = self.get_current_pose()
        if pose is None:
            log.fatal('无法获取当前位姿'); return

        start_x, start_y, start_yaw = pose
        log.info(f'起点: ({start_x:.3f}, {start_y:.3f}, yaw={start_yaw:.3f})')

        fwd_x = start_x + FORWARD_DISTANCE * math.cos(start_yaw)
        fwd_y = start_y + FORWARD_DISTANCE * math.sin(start_yaw)
        log.info(f'目标: ({fwd_x:.3f}, {fwd_y:.3f})')

        self._set_aim(False)

        for i in range(NUM_ROUNDS):
            log.info(f'===== 第 {i+1}/{NUM_ROUNDS} 轮 =====')

            log.info('--- 前进, 云台扫描 ON ---')
            self._scan_on()
            ok = self.navigate_to(fwd_x, fwd_y, start_yaw)
            if not ok:
                log.warn('前进导航失败, 继续')

            log.info(f'--- 到达, 停留 {DWELL_SECS}s ---')
            self._stop_chassis()
            self._spin_ros(DWELL_SECS)

            log.info('--- 返回起点 ---')
            ok = self.navigate_to(start_x, start_y, start_yaw)
            if not ok:
                log.warn('返回导航失败, 继续')

            log.info(f'--- 回到起点, 停留 {DWELL_SECS}s ---')
            self._stop_chassis()
            self._spin_ros(DWELL_SECS)

        log.info('===== 测试完成 =====')


def main(args=None):
    rclpy.init(args=args)
    node = TestCompetitionSimple()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.get_logger().info('安全关闭')
        node._scan_off()
        node._set_aim(False)
        node._stop_chassis()
        node._spin_ros(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
