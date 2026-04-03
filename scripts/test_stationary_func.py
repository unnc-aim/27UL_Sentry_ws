#!/usr/bin/env python3
"""
Stationary function test: verify toggle and data flow for all control
subsystems WITHOUT requiring chassis movement.

Tests
-----
1. Gimbal scan ON / OFF       (/gimbal_scan_cmd, pb_rm_interfaces/GimbalCmd)
2. Auto-aim switch ON / OFF   (/auto_aim_switch, std_msgs/Int32)
3. Chassis spin ON / OFF      (/cmd_spin, example_interfaces/Float32)

Each test checks:
  - Topic publish & self-echo   (data pipeline operational)
  - External subscriber count   (downstream consumers connected)
"""

import math
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from example_interfaces.msg import Float32
from pb_rm_interfaces.msg import GimbalCmd
from std_msgs.msg import Int32


SPIN_TEST_SPEED = 3.0       # rad/s for 小陀螺 ON test
GIMBAL_YAW_SPEED = 1.0      # rad/s for gimbal scan
HOLD_DURATION = 3.0          # seconds to hold each state for visual confirmation
DDS_DISCOVERY_WAIT = 2.0     # seconds to wait for DDS endpoint discovery
VERIFY_TIMEOUT = 2.0         # seconds to wait for self-echo verification
VERIFY_HZ = 20.0             # publish rate during verification


@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str = ''


@dataclass
class TestReport:
    results: list = field(default_factory=list)

    def add(self, name: str, passed: bool, detail: str = ''):
        self.results.append(TestResult(name, passed, detail))

    @property
    def all_passed(self):
        return all(r.passed for r in self.results)


class StationaryFuncTest(Node):
    def __init__(self):
        super().__init__('stationary_func_test')

        qos_be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._gimbal_pub = self.create_publisher(GimbalCmd, '/gimbal_scan_cmd', qos_be)
        self._aim_pub = self.create_publisher(Int32, '/auto_aim_switch', qos_rel)
        self._spin_pub = self.create_publisher(Float32, '/cmd_spin', qos_be)

        self._last_gimbal = None
        self._last_aim = None
        self._last_spin = None

        self._gimbal_sub = self.create_subscription(
            GimbalCmd, '/gimbal_scan_cmd', self._cb_gimbal, qos_be)
        self._aim_sub = self.create_subscription(
            Int32, '/auto_aim_switch', self._cb_aim, qos_rel)
        self._spin_sub = self.create_subscription(
            Float32, '/cmd_spin', self._cb_spin, qos_be)

        self.report = TestReport()

    # ── subscription callbacks ──────────────────────────────────────────

    def _cb_gimbal(self, msg):
        self._last_gimbal = msg

    def _cb_aim(self, msg):
        self._last_aim = msg

    def _cb_spin(self, msg):
        self._last_spin = msg

    # ── helpers ─────────────────────────────────────────────────────────

    def spin_for(self, duration: float):
        t0 = time.time()
        while (time.time() - t0) < duration:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _external_sub_count(self, publisher) -> int:
        """Subscriber count excluding our own self-subscription."""
        return max(0, publisher.get_subscription_count() - 1)

    def _verify_echo(self, get_last, check_fn, publish_fn,
                     timeout=VERIFY_TIMEOUT) -> bool:
        t0 = time.time()
        while (time.time() - t0) < timeout:
            publish_fn()
            rclpy.spin_once(self, timeout_sec=1.0 / VERIFY_HZ)
            last = get_last()
            if last is not None and check_fn(last):
                return True
        return False

    def _hold_publish(self, publish_fn, duration: float) -> int:
        t0 = time.time()
        count = 0
        while (time.time() - t0) < duration:
            publish_fn()
            rclpy.spin_once(self, timeout_sec=0.05)
            count += 1
        return count

    # ── publish helpers ─────────────────────────────────────────────────

    def _pub_gimbal(self, yaw_speed: float):
        msg = GimbalCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.yaw_type = GimbalCmd.VELOCITY
        msg.pitch_type = GimbalCmd.VELOCITY
        msg.velocity.yaw = yaw_speed
        msg.velocity.pitch = 0.0
        msg.velocity.yaw_min_range = -math.pi
        msg.velocity.yaw_max_range = math.pi
        msg.velocity.pitch_min_range = -0.3
        msg.velocity.pitch_max_range = 0.3
        self._gimbal_pub.publish(msg)

    def _pub_aim(self, on: bool):
        msg = Int32()
        msg.data = 1 if on else 0
        self._aim_pub.publish(msg)

    def _pub_spin(self, speed: float):
        msg = Float32()
        msg.data = speed
        self._spin_pub.publish(msg)

    # ── individual tests ────────────────────────────────────────────────

    def test_gimbal_scan(self):
        log = self.get_logger()
        ext = self._external_sub_count(self._gimbal_pub)
        log.info(f'gimbal_scan_cmd 外部订阅者: {ext}')
        self.report.add('gimbal_scan_cmd_subscribers',
                        ext > 0, f'{ext} external subscriber(s)')

        # ON
        log.info('--- 云台扫描 ON (yaw_speed=%.1f) ---' % GIMBAL_YAW_SPEED)
        self._last_gimbal = None
        on_ok = self._verify_echo(
            lambda: self._last_gimbal,
            lambda m: abs(m.velocity.yaw - GIMBAL_YAW_SPEED) < 0.01,
            lambda: self._pub_gimbal(GIMBAL_YAW_SPEED),
        )
        tag = 'PASS' if on_ok else 'FAIL'
        log.info(f'  [{tag}] 云台扫描 ON 自回环验证')
        self.report.add('gimbal_scan_on', on_ok)

        n = self._hold_publish(lambda: self._pub_gimbal(GIMBAL_YAW_SPEED), HOLD_DURATION)
        log.info(f'  持续发送 {n} 帧 ({HOLD_DURATION}s)')

        # OFF
        log.info('--- 云台扫描 OFF ---')
        self._last_gimbal = None
        off_ok = self._verify_echo(
            lambda: self._last_gimbal,
            lambda m: abs(m.velocity.yaw) < 0.01,
            lambda: self._pub_gimbal(0.0),
        )
        tag = 'PASS' if off_ok else 'FAIL'
        log.info(f'  [{tag}] 云台扫描 OFF 自回环验证')
        self.report.add('gimbal_scan_off', off_ok)

        self._hold_publish(lambda: self._pub_gimbal(0.0), 1.0)

    def test_autoaim(self):
        log = self.get_logger()
        ext = self._external_sub_count(self._aim_pub)
        log.info(f'auto_aim_switch 外部订阅者: {ext}')
        self.report.add('auto_aim_switch_subscribers',
                        ext > 0, f'{ext} external subscriber(s)')

        # ON
        log.info('--- 自瞄 ON ---')
        self._last_aim = None
        on_ok = self._verify_echo(
            lambda: self._last_aim,
            lambda m: m.data == 1,
            lambda: self._pub_aim(True),
        )
        tag = 'PASS' if on_ok else 'FAIL'
        log.info(f'  [{tag}] auto_aim_switch=1 自回环验证')
        self.report.add('autoaim_on', on_ok)

        self._hold_publish(lambda: self._pub_aim(True), HOLD_DURATION)

        # OFF
        log.info('--- 自瞄 OFF ---')
        self._last_aim = None
        off_ok = self._verify_echo(
            lambda: self._last_aim,
            lambda m: m.data == 0,
            lambda: self._pub_aim(False),
        )
        tag = 'PASS' if off_ok else 'FAIL'
        log.info(f'  [{tag}] auto_aim_switch=0 自回环验证')
        self.report.add('autoaim_off', off_ok)

        self._hold_publish(lambda: self._pub_aim(False), 1.0)

    def test_spin(self):
        log = self.get_logger()
        ext = self._external_sub_count(self._spin_pub)
        log.info(f'cmd_spin 外部订阅者: {ext}')
        self.report.add('cmd_spin_subscribers',
                        ext > 0, f'{ext} external subscriber(s)')

        # ON
        log.info('--- 小陀螺 ON (speed=%.1f rad/s) ---' % SPIN_TEST_SPEED)
        self._last_spin = None
        on_ok = self._verify_echo(
            lambda: self._last_spin,
            lambda m: abs(m.data - SPIN_TEST_SPEED) < 0.01,
            lambda: self._pub_spin(SPIN_TEST_SPEED),
        )
        tag = 'PASS' if on_ok else 'FAIL'
        log.info(f'  [{tag}] cmd_spin={SPIN_TEST_SPEED} 自回环验证')
        self.report.add('spin_on', on_ok)

        n = self._hold_publish(lambda: self._pub_spin(SPIN_TEST_SPEED), HOLD_DURATION + 2.0)
        log.info(f'  持续发送 {n} 帧 ({HOLD_DURATION + 2.0}s) — 可观察底盘是否旋转')

        # OFF
        log.info('--- 小陀螺 OFF ---')
        self._last_spin = None
        off_ok = self._verify_echo(
            lambda: self._last_spin,
            lambda m: abs(m.data) < 0.01,
            lambda: self._pub_spin(0.0),
        )
        tag = 'PASS' if off_ok else 'FAIL'
        log.info(f'  [{tag}] cmd_spin=0.0 自回环验证')
        self.report.add('spin_off', off_ok)

        self._hold_publish(lambda: self._pub_spin(0.0), 1.0)

    # ── summary ─────────────────────────────────────────────────────────

    def print_summary(self):
        log = self.get_logger()
        log.info('=' * 50)
        log.info('          静态功能测试结果汇总')
        log.info('=' * 50)
        for r in self.report.results:
            tag = 'PASS' if r.passed else 'FAIL'
            extra = f'  ({r.detail})' if r.detail else ''
            log.info(f'  [{tag}] {r.name}{extra}')
        log.info('-' * 50)
        if self.report.all_passed:
            log.info('  总结: 全部通过 ✓')
        else:
            failed = [r.name for r in self.report.results if not r.passed]
            log.warn(f'  总结: {len(failed)} 项未通过 — {", ".join(failed)}')
        log.info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = StationaryFuncTest()

    try:
        node.get_logger().info('===== 静态功能测试 (无需移动) =====')
        node.get_logger().info(f'等待 DDS 端点发现 ({DDS_DISCOVERY_WAIT}s)...')
        node.spin_for(DDS_DISCOVERY_WAIT)

        node.get_logger().info('===== STEP 1: 云台扫描 开/关 =====')
        node.test_gimbal_scan()

        node.get_logger().info('===== STEP 2: 自瞄检测 开/关 =====')
        node.test_autoaim()

        node.get_logger().info('===== STEP 3: 小陀螺 开/关 =====')
        node.test_spin()

        node.print_summary()
        node.get_logger().info('===== 测试结束 =====')

    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.get_logger().info('安全关闭: 所有输出归零')
        node._pub_gimbal(0.0)
        node._pub_aim(False)
        node._pub_spin(0.0)
        node.spin_for(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
