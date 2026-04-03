#!/usr/bin/env python3
"""
RMUL 2025 正赛控制器 — 遵循 test_simple_nav.py 通信模式

使用 NavigateToPose 逐点导航 (非 NavigateThroughPoses),
因为 Nav2 的 navigate_through_poses BT 内部 RemovePassedGoals
(radius=0.7m) 与 goal_checker (0.15m) 的容差不匹配,
会导致机器人永不停下。

状态机:
  INIT       → 等待 Nav2 / /map / TF 就绪
  WAIT_GAME  → 监听裁判系统, 云台慢速旋转
  NAVIGATE   → 逐点 NavigateToPose 到目标, 不开小陀螺/自瞄, 云台慢旋
  COMBAT     → SCAN + 自瞄 + 热量管理 (240 停 / 50 恢复)
  RETREAT    → HP <= 150 反向航点回家
  RECOVER    → 等待 HP (400 / 超时降级 350) 后重新出发

航点坐标系: SLAM 模式下 map 原点 = 机器人上电位置,
因此所有航点都是相对于出发点的坐标。
"""

import math
import time
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from dji_referee_protocol.msg import GameStatus, RobotHeat, RobotPerformance
from example_interfaces.msg import Float32
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from pb_rm_interfaces.msg import GimbalCmd
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener

# ── 比赛常量 ─────────────────────────────────────────────────────
GAME_STAGE_IN_GAME = 4

FORWARD_WAYPOINTS = [
    (6.0, 0.0, 0.0),
    (6.0, 5.25, 0.0),
    (3.0, 5.25, 0.0),
]
RETREAT_WAYPOINTS = [
    (6.0, 5.25, 0.0),
    (6.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
]

SCAN_SPEED_SLOW = 0.5
SCAN_SPEED_COMBAT = 1.0
SPIN_SPEED = 7.0              # 与 BT XML 一致

HP_RETREAT = 150
HP_FULL = 400
HP_RECOVER_ENOUGH = 350
RECOVER_PATIENCE = 30.0
HEAT_STOP = 240
HEAT_RESUME = 50

NAV_PER_WP_TIMEOUT = 60.0
READY_TIMEOUT = 120.0
REFEREE_TIMEOUT = 300.0


class Phase(Enum):
    INIT = auto()
    WAIT_GAME = auto()
    NAVIGATE = auto()
    COMBAT = auto()
    RETREAT = auto()
    RECOVER = auto()


class CompetitionController(Node):
    def __init__(self):
        super().__init__('competition_controller')

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
        self._spin_pub = self.create_publisher(Float32, '/cmd_spin', qos_be)
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._game_progress = 0
        self._hp = 400
        self._heat = 0
        self.create_subscription(
            GameStatus, '/referee/common/game_status', self._on_game, 10)
        self.create_subscription(
            RobotPerformance, '/referee/common/robot_performance', self._on_perf, 10)
        self.create_subscription(
            RobotHeat, '/referee/common/robot_heat', self._on_heat, 10)

        self._scanning = False
        self._scan_speed = SCAN_SPEED_SLOW
        self._desired_aim = False
        self._desired_spin = 0.0
        self._phase = Phase.INIT
        self._heat_exceeded = False

        self._aim_tick = 0
        self._spin_tick = 0
        self._tick_timer = self.create_timer(1.0 / 20.0, self._tick)

    # ══════════════════════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════════════════════

    def _on_map(self, msg):
        if not self._map_ok:
            self.get_logger().info(
                f'Map: {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution:.3f} m/cell')
        self._map_ok = True

    def _on_game(self, msg):
        if msg.game_progress != self._game_progress:
            self.get_logger().info(
                f'裁判: game_progress {self._game_progress} → {msg.game_progress}')
        self._game_progress = msg.game_progress

    def _on_perf(self, msg):
        self._hp = msg.current_hp

    def _on_heat(self, msg):
        self._heat = msg.shooter_17mm_barrel_heat

    def _tick(self):
        if self._scanning:
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

        self._aim_tick += 1
        if self._aim_tick >= 10:
            self._aim_tick = 0
            a = Int32()
            a.data = 1 if self._desired_aim else 0
            self._aim_pub.publish(a)

        self._spin_tick += 1
        if self._spin_tick >= 10:                 # ~2 Hz
            self._spin_tick = 0
            s = Float32()
            s.data = self._desired_spin
            self._spin_pub.publish(s)

    # ══════════════════════════════════════════════════════════════
    #  Publish helpers
    # ══════════════════════════════════════════════════════════════

    def _stop_chassis(self):
        self._vel_pub.publish(Twist())

    def _set_aim(self, on: bool):
        self._desired_aim = on
        m = Int32(); m.data = 1 if on else 0
        self._aim_pub.publish(m)
        self.get_logger().info(f'auto_aim_switch = {m.data}')

    def _set_spin(self, speed: float):
        self._desired_spin = speed
        m = Float32(); m.data = speed
        self._spin_pub.publish(m)

    def _scan_on(self, speed: float):
        self._scan_speed = speed
        self._scanning = True

    def _scan_off(self):
        self._scanning = False
        g = GimbalCmd()
        g.header.stamp = self.get_clock().now().to_msg()
        g.yaw_type = GimbalCmd.VELOCITY
        g.pitch_type = GimbalCmd.VELOCITY
        self._gimbal_pub.publish(g)

    # ══════════════════════════════════════════════════════════════
    #  Utilities
    # ══════════════════════════════════════════════════════════════

    def _spin_ros(self, secs: float):
        t0 = time.time()
        while (time.time() - t0) < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _game_on(self) -> bool:
        return self._game_progress == GAME_STAGE_IN_GAME

    def _tf_ok(self) -> bool:
        try:
            self._tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return True
        except TransformException:
            return False

    # ══════════════════════════════════════════════════════════════
    #  NavigateToPose 单点导航 (与 test_simple_nav.py 一致)
    # ══════════════════════════════════════════════════════════════

    def _navigate_to(self, x, y, yaw, timeout=NAV_PER_WP_TIMEOUT):
        log = self.get_logger()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        log.info(f'导航 → ({x:.2f}, {y:.2f})')

        fut = self._nav_ac.send_goal_async(goal, feedback_callback=self._nav_fb)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        gh = fut.result()
        if not gh.accepted:
            log.error('目标被拒绝')
            self._stop_chassis()
            return False

        log.info('目标已接受')
        fut_res = gh.get_result_async()
        t0 = time.time()
        while not fut_res.done():
            rclpy.spin_once(self, timeout_sec=0.05)

            if time.time() - t0 > timeout:
                log.warn(f'超时 ({timeout}s), 取消')
                self._cancel_nav(gh, fut_res)
                return False

            if not self._game_on():
                log.warn('比赛结束, 取消导航')
                self._cancel_nav(gh, fut_res)
                return False

        self._stop_chassis()

        status = fut_res.result().status
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(status, str(status))
        log.info(f'  结果: {status_name}')
        return status == GoalStatus.STATUS_SUCCEEDED

    def _cancel_nav(self, gh, fut_res):
        gh.cancel_goal_async()
        tc = time.time()
        while not fut_res.done() and (time.time() - tc) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._stop_chassis()

    def _nav_fb(self, fb_msg):
        pos = fb_msg.feedback.current_pose.pose.position
        self.get_logger().info(
            f'  pos: ({pos.x:.2f}, {pos.y:.2f})',
            throttle_duration_sec=3.0)

    # ── 多航点: 逐点 NavigateToPose ──────────────────────────────

    def _navigate_waypoints(self, waypoints, abort_on_hp_low=False):
        """逐个航点调用 NavigateToPose, 每个航点间检查 HP/比赛状态."""
        log = self.get_logger()
        tag = ' → '.join(f'({x:.1f},{y:.1f})' for x, y, _ in waypoints)
        log.info(f'航线: {tag}')

        for i, (x, y, yaw) in enumerate(waypoints):
            if not self._game_on():
                log.warn('比赛结束')
                return False

            if abort_on_hp_low and self._hp <= HP_RETREAT:
                log.warn(f'HP={self._hp} <= {HP_RETREAT}, 中断导航')
                self._stop_chassis()
                return False

            log.info(f'  航点 {i+1}/{len(waypoints)}')
            ok = self._navigate_to(x, y, yaw)
            if not ok:
                return False

        return True

    # ══════════════════════════════════════════════════════════════
    #  Phase handlers
    # ══════════════════════════════════════════════════════════════

    def _ph_init(self):
        log = self.get_logger()
        log.info('===== INIT =====')

        log.info('等待 /map ...')
        t0 = time.time()
        while not self._map_ok:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - t0 > READY_TIMEOUT:
                log.error('Map 超时'); return False
        log.info('Map OK')

        log.info('等待 TF (map → base_footprint) ...')
        while not self._tf_ok():
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

        self._phase = Phase.WAIT_GAME
        return True

    def _ph_wait_game(self):
        log = self.get_logger()
        log.info('===== WAIT_GAME =====')
        self._scan_on(SCAN_SPEED_SLOW)
        self._set_aim(False)
        self._set_spin(0.0)
        self._stop_chassis()

        log.info(f'等待裁判 IN_GAME (超时 {REFEREE_TIMEOUT}s) ...')
        t0 = time.time()
        last_log_sec = 0
        while self._game_progress != GAME_STAGE_IN_GAME:
            rclpy.spin_once(self, timeout_sec=0.2)
            elapsed = time.time() - t0
            if elapsed > REFEREE_TIMEOUT:
                log.error(f'裁判超时, progress={self._game_progress}')
                return False
            sec = int(elapsed)
            if sec > 0 and sec % 30 == 0 and sec != last_log_sec:
                last_log_sec = sec
                log.info(f'  等待 {sec}s ...')

        log.info('裁判: IN_GAME!')

        if not self._map_ok:
            log.error('Map 丢失'); return False
        if not self._tf_ok():
            log.error('TF 丢失'); return False

        log.info('Map / TF 正常, 开始!')
        self._phase = Phase.NAVIGATE
        return True

    def _ph_navigate(self):
        log = self.get_logger()
        log.info('===== NAVIGATE → 目标点 =====')
        self._set_aim(False)
        self._set_spin(0.0)
        self._scan_on(SCAN_SPEED_SLOW)

        ok = self._navigate_waypoints(FORWARD_WAYPOINTS, abort_on_hp_low=True)

        if ok:
            log.info('到达目标点!')
            self._stop_chassis()
            self._phase = Phase.COMBAT
            return True

        if self._hp <= HP_RETREAT:
            log.warn(f'导航中 HP={self._hp}, 转入撤退')
            self._stop_chassis()
            self._phase = Phase.RETREAT
            return True

        if self._hp <= 0:
            log.warn('导航中 HP 归零')
            self._stop_chassis()
            self._phase = Phase.RECOVER
            return True

        if not self._game_on():
            self._stop_chassis()
            return False

        log.warn('导航失败, 2s 后重试')
        self._stop_chassis()
        self._spin_ros(2.0)
        return self._game_on()

    def _ph_combat(self):
        log = self.get_logger()
        log.info('===== COMBAT =====')
        self._stop_chassis()
        self._set_spin(SPIN_SPEED)
        self._scan_on(SCAN_SPEED_COMBAT)
        self._set_aim(True)
        self._heat_exceeded = False

        while self._game_on():
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self._heat_exceeded and self._heat >= HEAT_STOP:
                log.info(f'热量 {self._heat} >= {HEAT_STOP}, 暂停射击')
                self._heat_exceeded = True
                self._set_aim(False)
            elif self._heat_exceeded and self._heat <= HEAT_RESUME:
                log.info('热量已降, 恢复射击')
                self._heat_exceeded = False
                self._set_aim(True)

            if self._hp <= HP_RETREAT:
                log.info(f'HP {self._hp} <= {HP_RETREAT}, 撤退!')
                self._set_aim(False)
                self._phase = Phase.RETREAT
                return True

        return False

    def _ph_retreat(self):
        log = self.get_logger()
        log.info('===== RETREAT → 基地 =====')
        self._set_aim(False)
        self._set_spin(0.0)
        self._scan_on(SCAN_SPEED_SLOW)

        if self._navigate_waypoints(RETREAT_WAYPOINTS):
            log.info('到达基地!')
            self._stop_chassis()
            self._phase = Phase.RECOVER
            return True

        if self._hp <= 0:
            log.warn('撤退中 HP 归零')
            self._stop_chassis()
            self._phase = Phase.RECOVER
            return True

        log.warn('撤退失败, 2s 后重试')
        self._stop_chassis()
        self._spin_ros(2.0)
        return self._game_on()

    def _ph_recover(self):
        log = self.get_logger()
        log.info('===== RECOVER =====')
        self._set_aim(False)
        self._set_spin(SPIN_SPEED)
        self._stop_chassis()
        self._scan_on(SCAN_SPEED_SLOW)

        t_recover_start = time.time()
        hp_snapshot = self._hp

        while self._game_on():
            rclpy.spin_once(self, timeout_sec=0.5)

            if self._hp >= HP_FULL:
                log.info(f'HP 满 ({self._hp}), 出发!')
                self._phase = Phase.NAVIGATE
                return True

            elapsed = time.time() - t_recover_start
            if self._hp >= HP_RECOVER_ENOUGH and elapsed > RECOVER_PATIENCE:
                log.info(
                    f'HP {self._hp} >= {HP_RECOVER_ENOUGH} '
                    f'且等待 {elapsed:.0f}s, 提前出发')
                self._phase = Phase.NAVIGATE
                return True

            if self._hp < hp_snapshot:
                hp_snapshot = self._hp
                t_recover_start = time.time()

            if self._hp <= 0:
                log.info('HP=0, 等待复活 ...', throttle_duration_sec=5.0)
                t_recover_start = time.time()
            else:
                log.info(
                    f'  HP: {self._hp}/{HP_FULL}'
                    f' ({elapsed:.0f}s)',
                    throttle_duration_sec=5.0)

        return False

    # ══════════════════════════════════════════════════════════════
    #  主循环
    # ══════════════════════════════════════════════════════════════

    def run(self):
        handlers = {
            Phase.INIT: self._ph_init,
            Phase.WAIT_GAME: self._ph_wait_game,
            Phase.NAVIGATE: self._ph_navigate,
            Phase.COMBAT: self._ph_combat,
            Phase.RETREAT: self._ph_retreat,
            Phase.RECOVER: self._ph_recover,
        }

        try:
            while True:
                h = handlers.get(self._phase)
                if not h or not h():
                    break
                rclpy.spin_once(self, timeout_sec=0.01)

            self.get_logger().info('===== 比赛结束 =====')
        except KeyboardInterrupt:
            self.get_logger().info('用户中断')
        finally:
            self.get_logger().info('安全关闭: 输出归零')
            self._scan_off()
            self._set_aim(False)
            self._set_spin(0.0)
            self._stop_chassis()
            self._spin_ros(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
