import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from custom_msgs.msg import ReadDJIRC, WriteDJIMotor, ReadDJIMotor

# 引入我们在云台里用到的 PID 工具
from infantry_controller.pid import PID

class FireController(Node):
    STATE_IDLE = 0
    STATE_LOADING = 1
    STATE_READY = 2
    # 移除了 STATE_FIRING，因为串级 PID 会自动顺滑且快速地完成多圈位置跟踪

    def __init__(self):
        super().__init__('fire_controller')
        self._declare_params()

        # --- 拨盘参数 ---
        # 保持与线下代码一致的正向增量
        self.ONE_BULLET_ECD = 36.0 * 8192.0 / 8.0
        self.FRICTION_SPEED_TARGET = 6500

        # --- PID 初始化 (参数源自原版 gimbal_task.h) ---
        # 拨盘位置环 (输出：目标转速 RPM)
        self.pid_angle = PID(
            kp=0.4, ki=0.0, kd=7.0,
            max_out=13000.0, max_iout=1500.0
        )
        # 拨盘速度环 (输出：目标力矩电流)
        self.pid_speed = PID(
            kp=5.0, ki=0.01, kd=0.0,
            max_out=10000.0, max_iout=1000.0
        )

        # --- 状态变量 ---
        self.rc_data = None
        self.rc_connected = False

        self.feeder_state = self.STATE_IDLE
        self.is_friction_on = False
        self.burst_mode = False
        self.last_switch_right = 0

        self.trigger_target_ecd = 0.0
        self.trigger_has_fired = False
        self.last_shot_time = 0.0
        self.autoaim_enable_state = False

        # --- 多圈编码器解算变量 ---
        self.motor3_current = 0
        self.motor3_rpm = 0
        self.motor3_ecd = 0.0       # 单圈 (0-8191)
        self.total_ecd = 0.0        # 连续多圈累加值
        self.motor_initialized = False

        self.diag_counter = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        topic_rc = str(self.get_parameter('topic_rc_read').value or '/ecat/sn4587585/app1/read')
        topic_fire_write = str(self.get_parameter('topic_fire_write').value or '/ecat/sn4587585/app3/write')
        topic_fire_read = str(self.get_parameter('topic_fire_read').value or '/ecat/sn4587585/app3/read')
        topic_autoaim_enable = str(
            self.get_parameter('topic_autoaim_enable').value or '/sp_vision/autoaim_enable')

        self.sub_rc = self.create_subscription(ReadDJIRC, topic_rc, self.cb_rc, qos)
        self.sub_motor = self.create_subscription(ReadDJIMotor, topic_fire_read, self.cb_motor_fb, qos)
        self.pub_fire = self.create_publisher(WriteDJIMotor, topic_fire_write, qos)
        self.pub_autoaim_enable = self.create_publisher(Bool, topic_autoaim_enable, qos)

        self.timer = self.create_timer(0.001, self.control_loop)
        self._publish_autoaim_enable(False, force=True)
        self.get_logger().info("Fire Controller Started @ 1000Hz [Multi-turn Cascade PID]")

    def _declare_params(self):
        self.declare_parameter('topic_rc_read', '/ecat/sn4587585/app1/read')
        self.declare_parameter('topic_fire_write', '/ecat/sn4587585/app3/write')
        self.declare_parameter('topic_fire_read', '/ecat/sn4587585/app3/read')
        self.declare_parameter('topic_autoaim_enable', '/sp_vision/autoaim_enable')
        
        # 射击间隔（毫秒）
        self.declare_parameter('shot_period_ms', 100.0)
        self.declare_parameter('load_current_threshold', 500)
        self.declare_parameter('load_speed_ecd', 2.5)

    def cb_rc(self, msg):
        self.rc_data = msg
        self.rc_connected = (msg.online == 1)

    def cb_motor_fb(self, msg):
        """多圈绝对编码器解算逻辑"""
        self.motor3_current = msg.motor3_current
        self.motor3_rpm = msg.motor3_rpm
        current_ecd = float(msg.motor3_ecd)

        if not self.motor_initialized:
            self.motor3_ecd = current_ecd
            self.total_ecd = current_ecd
            self.trigger_target_ecd = current_ecd
            self.motor_initialized = True
        else:
            # 计算最短路径差值，处理 0 -> 8191 或 8191 -> 0 的跳变
            delta = current_ecd - self.motor3_ecd
            if delta > 4096:
                delta -= 8192
            elif delta < -4096:
                delta += 8192
            
            self.total_ecd += delta
            self.motor3_ecd = current_ecd

    def control_loop(self):
        if not self.rc_connected or not self.rc_data or not self.motor_initialized:
            self._publish_autoaim_enable(False)
            self.stop_all()
            return

        sw_right = self.rc_data.right_switch  # 1=Up, 3=Mid, 2=Down

        load_threshold = int(self.get_parameter('load_current_threshold').value or 500)
        load_speed = float(self.get_parameter('load_speed_ecd').value or 2.5)
        shot_period = float(self.get_parameter('shot_period_ms').value or 100.0) / 1000.0

        # ================= 1. 状态机与档位逻辑 =================
        if sw_right == 2:
            self.is_friction_on = False
            self.feeder_state = self.STATE_IDLE
            self.trigger_target_ecd = self.total_ecd  # 锁住当前位置
            friction_cmd = 0
            self._publish_autoaim_enable(False)
        else:
            self.is_friction_on = True
            friction_cmd = self.FRICTION_SPEED_TARGET

            if self.last_switch_right == 2 and sw_right == 3:
                self.burst_mode = not self.burst_mode
                mode_str = "BURST" if self.burst_mode else "SINGLE"
                self.feeder_state = self.STATE_LOADING
                self._publish_autoaim_enable(self.burst_mode)
                self.get_logger().info(f"Friction ON. Loading chamber... Mode: {mode_str}")

        self.last_switch_right = sw_right

        # ================= 2. 拨盘火控逻辑 =================
        if self.feeder_state == self.STATE_LOADING:
            # 缓慢推进直到接触子弹阻力增大
            if abs(self.motor3_current) > load_threshold:
                self.feeder_state = self.STATE_READY
                self.get_logger().info(f"Bullet loaded! Current: {self.motor3_current}mA. Ready to fire.")
            else:
                self.trigger_target_ecd += load_speed

        elif self.feeder_state == self.STATE_READY:
            should_fire = (sw_right == 1)

            if should_fire:
                now = self.get_clock().now().nanoseconds / 1e9
                # 核心机制：只有当上一发子弹转得差不多了（误差小于8192 即一圈），才允许叠加下一发的指令
                # 这还原了 C++ 代码中的: if (abs(target - total_ecd) < 8192) { target += ONE_BULLET_ECD; }
                if abs(self.trigger_target_ecd - self.total_ecd) < 8192:
                    if self.burst_mode:
                        if (now - self.last_shot_time) > shot_period:
                            self.trigger_target_ecd += self.ONE_BULLET_ECD
                            self.last_shot_time = now
                    else:
                        if not self.trigger_has_fired:
                            self.trigger_target_ecd += self.ONE_BULLET_ECD
                            self.trigger_has_fired = True
            else:
                self.trigger_has_fired = False

        # ================= 3. 串级 PID 计算 (核心更新点) =================
        if self.feeder_state == self.STATE_IDLE:
            # IDLE状态下，停止输出力矩
            motor3_torque = 0.0
            self.pid_angle.reset()
            self.pid_speed.reset()
        else:
            # 位置环 (目标位置 - 连续多圈位置)
            angle_error = self.trigger_target_ecd - self.total_ecd
            target_rpm = self.pid_angle.update(angle_error)

            # 速度环 (目标转速 - 实际转速)
            speed_error = target_rpm - self.motor3_rpm
            motor3_torque = self.pid_speed.update(speed_error)

        # ================= 4. 发布控制指令 =================
        msg = WriteDJIMotor()

        # 上下摩擦轮
        msg.motor1_enable = 1
        msg.motor1_cmd = int(friction_cmd)
        msg.motor2_enable = 1
        msg.motor2_cmd = int(-friction_cmd)

        # 拨盘电机发送计算出的力矩电流，而不是位置值！
        msg.motor3_enable = 1 if self.feeder_state != self.STATE_IDLE else 0
        msg.motor3_cmd = int(motor3_torque)

        # 诊断日志
        self.diag_counter += 1
        if self.diag_counter >= 500:
            self.diag_counter = 0
            self.get_logger().info(
                f"[DIAG] state={self.feeder_state} sw={sw_right} "
                f"target={self.trigger_target_ecd:.1f} total_ecd={self.total_ecd:.1f} "
                f"torque={motor3_torque:.1f} rpm={self.motor3_rpm} "
                f"cur={self.motor3_current}mA burst={self.burst_mode}"
            )

        self.pub_fire.publish(msg)

    def stop_all(self):
        msg = WriteDJIMotor()
        self.pub_fire.publish(msg)

    def _publish_autoaim_enable(self, enabled: bool, force: bool = False):
        if (not force) and self.autoaim_enable_state == enabled:
            return
        self.autoaim_enable_state = enabled
        msg = Bool()
        msg.data = enabled
        self.pub_autoaim_enable.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FireController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()