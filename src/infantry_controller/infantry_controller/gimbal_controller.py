"""
云台控制器模块

本模块实现步兵机器人的云台控制，包括 Pitch 轴和 Yaw 轴的控制。
Pitch 轴使用 DJI 电机的位置模式，Yaw 轴使用 LK 电机的力矩模式（级联 PID）。

Classes:
    GimbalController: 云台控制器节点
    
Functions:
    get_yaw_from_quaternion: 从四元数提取 Yaw 角
    get_pitch_from_quaternion: 从四元数提取 Pitch 角
    clamp: 数值限幅函数
    main: 主函数入口
"""
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from custom_msgs.msg import (  # type: ignore[reportMissingImports]
    ReadDJIMotor, WriteDJIMotor, ReadDJIRC, ReadLkMotor,
    WriteLkMotorTorqueControl)
from sp_msgs.msg import AutoAimCommandMsg
from infantry_controller.pid import PID


def get_yaw_from_quaternion(q) -> float:
    """
    从四元数提取 Yaw 角（偏航角）

    Args:
        q: 四元数对象，包含 w, x, y, z 属性

    Returns:
        float: Yaw 角（弧度）
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def get_pitch_from_quaternion(q) -> float:
    """
    从四元数提取 Pitch 角（俯仰角）

    Args:
        q: 四元数对象，包含 w, x, y, z 属性

    Returns:
        float: Pitch 角（弧度）
    """
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        return math.copysign(math.pi / 2, sinp)
    return math.asin(sinp)


def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    数值限幅函数

    Args:
        value: 输入值
        min_value: 最小值
        max_value: 最大值

    Returns:
        float: 限幅后的值
    """
    return max(min_value, min(max_value, value))


class GimbalController(Node):
    """
    云台控制器节点

    实现 Pitch 和 Yaw 两轴云台的闭环控制。
    - Pitch 轴：DJI 电机位置模式
    - Yaw 轴：LK 电机力矩模式（位置-速度级联 PID）

    控制频率：1000Hz

    Attributes:
        pid_yaw_pos (PID): Yaw 轴位置环 PID 控制器
        pid_yaw_spd (PID): Yaw 轴速度环 PID 控制器
        target_pitch_deg (float): 目标 Pitch 角度（度）
        target_yaw_rad (float): 目标 Yaw 角度（弧度）
        imu_pitch_rad (float): IMU 测量的 Pitch 角度（弧度）
        imu_yaw_rad (float): IMU 测量的 Yaw 角度（弧度）
        imu_gyro_z (float): IMU 测量的 Z 轴角速度（rad/s）
        yaw_motor_speed (float): Yaw 电机速度（rad/s）
        yaw_motor_pos (float): Yaw 电机位置（弧度）
        rc_data: 遥控器数据
        rc_connected (bool): 遥控器连接状态
    """

    def __init__(self) -> None:
        """
        初始化云台控制器节点

        设置参数、PID 控制器、通信接口和控制定时器。
        """
        super().__init__('gimbal_controller')

        # 1. 声明参数
        self._declare_params()

        # 2. 初始化 PID (使用初始参数)
        self.pid_yaw_pos = PID(
            float(self.get_parameter('yaw_pos_kp').value or 15.0),
            float(self.get_parameter('yaw_pos_ki').value or 0.0),
            float(self.get_parameter('yaw_pos_kd').value or 0.5),
            20.0, 5.0  # Max output (rad/s), Max I
        )
        self.pid_yaw_spd = PID(
            float(self.get_parameter('yaw_spd_kp').value or 20.0),
            float(self.get_parameter('yaw_spd_ki').value or 0.1),
            float(self.get_parameter('yaw_spd_kd').value or 0.0),
            2000.0, 500.0  # Max Torque, Max I
        )

        # 3. 状态变量
        self.target_pitch_deg = 0.0
        self.target_yaw_rad = 0.0
        self.imu_pitch_rad = 0.0
        self.imu_yaw_rad = 0.0
        self.imu_gyro_z = 0.0
        self.yaw_motor_speed = 0.0
        self.yaw_motor_pos = 0.0
        self.rc_data = None
        self.rc_connected = False
        self.autoaim_enabled = False
        self.autoaim_control = False
        self.autoaim_yaw = 0.0
        self.autoaim_pitch = 0.0
        self.autoaim_last_msg_time = 0.0
        self.autoaim_log_counter = 0

        # 4. 注册参数回调 (实现动态调参)
        self.add_on_set_parameters_callback(self.parameters_callback)

        # 5. 通信接口
        qos_best_effort = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 读取Topic参数
        topic_rc = self.get_parameter(
            'topic_rc_read').get_parameter_value().string_value
        topic_imu = self.get_parameter(
            'topic_imu_read').get_parameter_value().string_value
        topic_pitch_w = self.get_parameter(
            'topic_pitch_write').get_parameter_value().string_value
        topic_pitch_r = self.get_parameter(
            'topic_pitch_read').get_parameter_value().string_value
        topic_yaw_w = self.get_parameter(
            'topic_yaw_write').get_parameter_value().string_value
        topic_yaw_r = self.get_parameter(
            'topic_yaw_read').get_parameter_value().string_value
        topic_autoaim_cmd = self.get_parameter(
            'topic_autoaim_cmd').get_parameter_value().string_value
        topic_autoaim_enable = self.get_parameter(
            'topic_autoaim_enable').get_parameter_value().string_value

        self.sub_rc = self.create_subscription(
            ReadDJIRC, topic_rc, self.cb_rc, qos_best_effort)
        self.sub_imu = self.create_subscription(
            Imu, topic_imu, self.cb_imu, qos_best_effort)
        self.sub_pitch = self.create_subscription(
            ReadDJIMotor, topic_pitch_r, self.cb_pitch_fb, qos_best_effort)
        self.sub_yaw = self.create_subscription(
            ReadLkMotor, topic_yaw_r, self.cb_yaw_fb, qos_best_effort)
        self.sub_autoaim_cmd = self.create_subscription(
            AutoAimCommandMsg, topic_autoaim_cmd, self.cb_autoaim_command, qos_best_effort)
        self.sub_autoaim_enable = self.create_subscription(
            Bool, topic_autoaim_enable, self.cb_autoaim_enable, qos_best_effort)

        self.pub_pitch = self.create_publisher(
            WriteDJIMotor, topic_pitch_w, qos_best_effort)
        self.pub_yaw = self.create_publisher(
            WriteLkMotorTorqueControl, topic_yaw_w, qos_best_effort)

        # 6. 提升频率到 1000Hz (0.001s)
        self.timer = self.create_timer(0.001, self.control_loop)

        self.get_logger().info("Gimbal Controller Started @ 1000Hz with Parameter Callbacks")

    def _declare_params(self) -> None:
        """
        声明所有 ROS2 参数

        包括话题名称、限位参数、鼠标灵敏度和 PID 参数。
        """
        # Topics
        self.declare_parameter('topic_rc_read', '/ecat/sn4587585/app1/read')
        self.declare_parameter('topic_imu_read', '/ecat/sn4653090/app2/read')
        self.declare_parameter('topic_pitch_write',
                               '/ecat/sn4587585/app4/write')
        self.declare_parameter('topic_pitch_read', '/ecat/sn4587585/app4/read')
        self.declare_parameter('topic_yaw_write', '/ecat/sn4587586/app3/write')
        self.declare_parameter('topic_yaw_read', '/ecat/sn4653090/app3/read')
        self.declare_parameter('topic_autoaim_cmd',
                               '/sp_vision/autoaim_command')
        self.declare_parameter('topic_autoaim_enable',
                               '/sp_vision/autoaim_enable')
        self.declare_parameter('autoaim_timeout_s', 0.2)

        # Limits & Offsets
        self.declare_parameter('pitch_center_ecd', 4600)
        self.declare_parameter('pitch_min_deg', -15.0)
        self.declare_parameter('pitch_max_deg', 30.0)
        self.declare_parameter('mouse_sensitivity', 1.0)

        # PID Params
        self.declare_parameter('yaw_pos_kp', 15.0)
        self.declare_parameter('yaw_pos_ki', 0.0)
        self.declare_parameter('yaw_pos_kd', 0.5)

        self.declare_parameter('yaw_spd_kp', 20.0)
        self.declare_parameter('yaw_spd_ki', 0.1)
        self.declare_parameter('yaw_spd_kd', 0.0)

    def parameters_callback(self, params: list[Parameter]) -> SetParametersResult:
        """
        实时处理参数更新回调

        Args:
            params: 参数列表

        Returns:
            SetParametersResult: 参数更新结果
        """
        success = True
        for param in params:
            if param.name == 'yaw_pos_kp':
                self.pid_yaw_pos.update_params(kp=param.value)
            elif param.name == 'yaw_pos_ki':
                self.pid_yaw_pos.update_params(ki=param.value)
            elif param.name == 'yaw_pos_kd':
                self.pid_yaw_pos.update_params(kd=param.value)
            elif param.name == 'yaw_spd_kp':
                self.pid_yaw_spd.update_params(kp=param.value)
            elif param.name == 'yaw_spd_ki':
                self.pid_yaw_spd.update_params(ki=param.value)
            elif param.name == 'yaw_spd_kd':
                self.pid_yaw_spd.update_params(kd=param.value)
            elif param.name == 'pitch_center_ecd':
                # 直接更新内部变量，下一次循环生效
                pass
            # 记录日志
            self.get_logger().info(
                f'Updated parameter {param.name} to {param.value}')

        return SetParametersResult(successful=success)

    def cb_rc(self, msg: ReadDJIRC) -> None:
        """
        遥控器数据回调

        Args:
            msg: 遥控器数据消息
        """
        self.rc_data = msg
        self.rc_connected = (msg.online == 1)

    def cb_imu(self, msg: Imu) -> None:
        """
        IMU 数据回调

        Args:
            msg: IMU 数据消息
        """
        self.imu_pitch_rad = get_pitch_from_quaternion(msg.orientation)
        self.imu_yaw_rad = get_yaw_from_quaternion(msg.orientation)
        self.imu_gyro_z = msg.angular_velocity.z

    def cb_pitch_fb(self, msg: ReadDJIMotor) -> None:
        """
        Pitch 电机反馈回调

        Args:
            msg: DJI 电机反馈消息
        """
        pass

    def cb_yaw_fb(self, msg: ReadLkMotor) -> None:
        """
        Yaw 电机反馈回调

        Args:
            msg: LK 电机反馈消息
        """
        # 假设 speed 是 deg/s 或类似单位，转为 rad/s
        self.yaw_motor_speed = math.radians(msg.speed)
        self.yaw_motor_pos = (msg.encoder / 65535.0) * 2 * math.pi

    def cb_autoaim_enable(self, msg: Bool) -> None:
        """自瞄使能开关回调。"""
        self.autoaim_enabled = bool(msg.data)

    def cb_autoaim_command(self, msg: AutoAimCommandMsg) -> None:
        """接收视觉侧自瞄输出（世界系 yaw/pitch）。"""
        self.autoaim_control = bool(msg.control)
        self.autoaim_yaw = float(msg.yaw)
        self.autoaim_pitch = float(msg.pitch)
        self.autoaim_last_msg_time = self.get_clock().now().nanoseconds / 1e9

    def control_loop(self) -> None:
        """
        主控制循环（1000Hz）

        执行云台的 Pitch 和 Yaw 控制，包括：
        1. 安全检查（遥控器连接、急停）
        2. 遥控器输入映射
        3. Pitch 轴位置控制
        4. Yaw 轴级联 PID 控制（位置环+速度环）
        """
        if not self.rc_connected or not self.rc_data:
            self.stop_head_motors()
            return

        # 左拨杆下档触发急停
        if self.rc_data.left_switch == 2:
            self.stop_head_motors()
            return

        # 获取最新的限位参数 (支持实时更新)
        pitch_min = self.get_parameter(
            'pitch_min_deg').get_parameter_value().double_value
        pitch_max = self.get_parameter(
            'pitch_max_deg').get_parameter_value().double_value
        pitch_center = self.get_parameter(
            'pitch_center_ecd').get_parameter_value().integer_value
        mouse_sensitivity = self.get_parameter(
            'mouse_sensitivity').get_parameter_value().double_value
        autoaim_timeout = self.get_parameter(
            'autoaim_timeout_s').get_parameter_value().double_value

        now_sec = self.get_clock().now().nanoseconds / 1e9
        vision_cmd_fresh = (
            now_sec - self.autoaim_last_msg_time) < autoaim_timeout
        use_autoaim = self.autoaim_enabled and self.autoaim_control and vision_cmd_fresh

        # ================= 鼠标映射（对齐 legacy） =================
        # left_right_offset = left_x*100 + limit(mouse_x*0.75, 100)
        # top_down_offset   = left_y*100 + limit(-mouse_y, 100)
        mouse_x = float(self.rc_data.mouse_x) * mouse_sensitivity
        mouse_y = float(self.rc_data.mouse_y) * mouse_sensitivity
        left_right_offset = self.rc_data.left_x * 100.0 + \
            clamp(mouse_x * 0.75, -100.0, 100.0)
        top_down_offset = self.rc_data.left_y * 100.0 + \
            clamp(mouse_y, -100.0, 100.0)

        if use_autoaim:
            self.target_yaw_rad = math.atan2(
                math.sin(self.autoaim_yaw), math.cos(self.autoaim_yaw))
            self.target_pitch_deg = -math.degrees(self.autoaim_pitch)
        else:
            # ================= Pitch Control (DJI Motor 4 Position Mode) =================
            # legacy: current_pitch += top_down_offset * 0.00005(rad)
            rc_pitch_delta = top_down_offset * 0.00005 * (180.0 / math.pi)
            self.target_pitch_deg += rc_pitch_delta

            # ================= Yaw Control (LK Motor Torque Mode) =================
            # legacy: client_control_offset -= left_right_offset * 10*pi*0.001*0.0025
            rc_yaw_delta = -left_right_offset * 10.0 * math.pi * 0.001 * 0.0025
            self.target_yaw_rad += rc_yaw_delta

            # 归一化 Target 到 -PI ~ PI
            self.target_yaw_rad = math.atan2(
                math.sin(self.target_yaw_rad), math.cos(self.target_yaw_rad))

        self.target_pitch_deg = max(
            pitch_min, min(self.target_pitch_deg, pitch_max))

        current_pitch_deg = -math.degrees(self.imu_pitch_rad)
        pitch_error_deg = self.target_pitch_deg - current_pitch_deg

        # 8192 units per 360 degrees
        ecd_offset = int(pitch_error_deg * (8192.0 / 360.0))
        pitch_cmd_ecd = pitch_center + ecd_offset

        pitch_msg = WriteDJIMotor()
        pitch_msg.motor4_enable = 1
        pitch_msg.motor4_cmd = int(pitch_cmd_ecd)
        self.pub_pitch.publish(pitch_msg)

        # >>>>>>>>>>>> AUTOAIM DEBUG LOG (throttled ~10Hz) <<<<<<<<<<<<
        if use_autoaim:
            self.autoaim_log_counter += 1
            if self.autoaim_log_counter >= 100:  # 1000Hz / 100 = 10Hz
                self.autoaim_log_counter = 0
                self.get_logger().info(
                    f"[AUTOAIM] vision_yaw={self.autoaim_yaw:.4f} vision_pitch={self.autoaim_pitch:.4f} | "
                    f"target_yaw={self.target_yaw_rad:.4f} target_pitch={self.target_pitch_deg:.2f}deg | "
                    f"imu_yaw={self.imu_yaw_rad:.4f} imu_pitch={math.degrees(self.imu_pitch_rad):.2f}deg | "
                    f"pitch_err={pitch_error_deg:.2f}deg pitch_cmd_ecd={pitch_cmd_ecd}")
        else:
            self.autoaim_log_counter = 0

        # --- 1. 位置环 ---
        pos_error = self.target_yaw_rad - self.imu_yaw_rad
        # 归一化误差
        pos_error = math.atan2(math.sin(pos_error), math.cos(pos_error))

        # 前馈角速度 (基于遥控器输入)
        # 0.0001 * 1000Hz = 0.1 rad/s per full stick range roughly
        target_ang_vel = -left_right_offset * (10.0 * math.pi * 0.001)

        # D项输入: 期望角速度 - 测量角速度
        d_input_pos = target_ang_vel - self.imu_gyro_z

        # PID 计算 (使用独立的 PID 工具类)
        pid_pos_out = self.pid_yaw_pos.update(pos_error, d_input=d_input_pos)

        # --- 2. 速度环 ---
        # 速度环误差 = (期望角速度 + 位置环输出) - 测量角速度
        spd_target = target_ang_vel + pid_pos_out
        spd_error = spd_target - self.imu_gyro_z

        # 计算力矩
        torque_cmd = self.pid_yaw_spd.update(spd_error)

        yaw_msg = WriteLkMotorTorqueControl()
        yaw_msg.enable = 1
        yaw_msg.torque = int(torque_cmd)
        self.pub_yaw.publish(yaw_msg)

    def stop_pitch(self) -> None:
        """
        停止 Pitch 电机
        """
        msg = WriteDJIMotor()
        msg.motor4_enable = 0
        msg.motor4_cmd = 0
        self.pub_pitch.publish(msg)

    def stop_yaw(self) -> None:
        """
        停止 Yaw 电机
        """
        msg = WriteLkMotorTorqueControl()
        msg.enable = 0
        msg.torque = 0
        self.pub_yaw.publish(msg)

    def stop_head_motors(self) -> None:
        """
        停止所有云台电机
        """
        self.stop_pitch()
        self.stop_yaw()


def main(args=None) -> None:
    """
    主函数入口

    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = GimbalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
