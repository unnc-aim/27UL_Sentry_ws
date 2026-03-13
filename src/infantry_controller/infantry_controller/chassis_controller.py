"""
底盘控制器模块

本模块实现步兵机器人的舵轮底盘控制，支持全向移动、小陀螺模式和云台跟随。

Classes:
    ChassisController: 底盘控制器节点
    
Functions:
    main: 主函数入口
"""
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray


from custom_msgs.msg import (  # type: ignore[reportMissingImports]
    ReadDJIMotor, WriteDJIMotor, ReadDJIRC, ReadLkMotor)
from infantry_controller.chassis_kinematics import SwerveKinematics


class ChassisController(Node):
    """
    舵轮底盘控制器节点
    
    实现四轮舵轮底盘的运动控制，支持：
    - 全向移动（前后、左右、旋转）
    - 小陀螺模式（自动旋转）
    - 云台跟随（底盘坐标系与云台坐标系转换）
    - 遥控器和键盘输入
    - 速度分档
    
    控制频率：500Hz
    
    Attributes:
        kinematics (SwerveKinematics): 运动学解算器
        current_steer_ecds (list[int]): 当前舵向电机编码器值 [FL, FR, BL, BR]
        gimbal_yaw_angle (float): 底盘相对于云台的角度（弧度）
        rc_data: 遥控器数据
        rc_connected (bool): 遥控器连接状态
        mode_rotate_enabled (bool): 小陀螺模式标志
        current_spd_mode (float): 底盘分档速度
        spin_spd (float): 小陀螺基准速度
        wheel_track (float): 轮距（米）
        wheel_base (float): 轴距（米）
        ecd_zeros (list[int]): 舵向电机零位偏移 [FL, FR, BL, BR]
    """
    
    def __init__(self) -> None:
        """
        初始化底盘控制器节点
        
        设置参数、运动学解算器、通信接口和控制定时器。
        """
        super().__init__('chassis_controller')

        # ================= 参数声明与加载 =================
        self._declare_and_load_params()

        # ================= 运动学初始化 =================
        # 静态加载，无动态回调，确保实时性
        self.kinematics = SwerveKinematics(
            wheel_track=self.wheel_track,
            wheel_base=self.wheel_base
        )
        self.get_logger().info(
            f"Geometry Loaded: Track={self.wheel_track}m, Base={self.wheel_base}m")

        # ================= 状态变量 =================
        # 舵向电机当前编码器值 [FL, FR, BL, BR]
        self.current_steer_ecds = [0, 0, 0, 0]

        # 底盘相对于云台的角度 (由 Yaw 电机编码器解算)
        # 编码器 0-65535 对应 0-2PI
        self.gimbal_yaw_angle = 0.0

        # RC 状态
        self.rc_data = None
        self.rc_connected = False

        # 控制模式标志
        self.mode_rotate_enabled = False  # 小陀螺模式标志
        self.last_switch_state = 0       # 拨杆边沿检测
        self.last_v_pressed = 0          # 键盘 V 边沿检测

        # 键盘映射状态（对齐 legacy 逻辑）
        self.current_spd_mode = 3000.0   # 底盘分档速度
        self.spin_spd = 3000.0           # 小陀螺基准速度
        self.referee_speed_scale = 1.0
        self.referee_fire_allowed = True
        self.referee_heat = 0.0
        self.referee_heat_limit = 0.0
        self.referee_power = 0.0
        self.referee_power_limit = 0.0
        self.referee_last_time = 0.0

        # ================= 通信接口 =================
        qos_best_effort = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # 1. 订阅遥控器
        self.sub_rc = self.create_subscription(
            ReadDJIRC,
            self.topic_rc_read,
            self.cb_rc,
            qos_best_effort
        )

        # 2. 订阅舵向电机反馈
        self.sub_steer = self.create_subscription(
            ReadDJIMotor,
            self.topic_steer_read,
            self.cb_steer_feedback,
            qos_best_effort
        )

        # 3. 订阅 Yaw 电机反馈 (用于确定底盘正方向)
        self.sub_yaw = self.create_subscription(
            ReadLkMotor,
            self.topic_yaw_read,
            self.cb_yaw_feedback,
            qos_best_effort
        )

        self.sub_referee_constraints = self.create_subscription(
            Float32MultiArray,
            self.topic_referee_constraints,
            self.cb_referee_constraints,
            qos_best_effort
        )

        # 4. 发布驱动指令
        self.pub_drive = self.create_publisher(
            WriteDJIMotor,
            self.topic_drive_write,
            qos_best_effort
        )

        # 5. 发布舵向指令
        self.pub_steer = self.create_publisher(
            WriteDJIMotor,
            self.topic_steer_write,
            qos_best_effort
        )

        # ================== 定时器设置 ==================
        # 设置控制频率为 500Hz (周期 0.002s)
        self.timer = self.create_timer(1.0/500.0, self.control_loop)

        self.get_logger().info("Chassis Controller Started @ 500Hz")

    def _declare_and_load_params(self) -> None:
        """
        加载 YAML 参数 - 静态加载
        
        声明并读取所有配置参数，包括话题名称、几何参数和电机零位偏移。
        """
        # --- Topic ---
        self.declare_parameter('topic_rc_read', '/ecat/sn4587585/app1/read')
        self.declare_parameter('topic_drive_write',
                               '/ecat/sn4587586/app1/write')
        self.declare_parameter('topic_steer_read', '/ecat/sn4587586/app2/read')
        self.declare_parameter('topic_steer_write',
                               '/ecat/sn4587586/app2/write')
        self.declare_parameter('topic_yaw_read', '/ecat/sn4587586/app3/read')
        self.declare_parameter('topic_referee_constraints', '/referee/constraints')
        self.declare_parameter('referee_timeout_s', 0.5)

        # --- Geometry ---
        self.declare_parameter('wheel_track', 0.4)
        self.declare_parameter('wheel_base', 0.4)

        # --- Offsets ---
        self.declare_parameter('offset_fl', 0)
        self.declare_parameter('offset_fr', 0)
        self.declare_parameter('offset_bl', 0)
        self.declare_parameter('offset_br', 0)

        # --- 读取参数值 ---
        self.topic_rc_read = self.get_parameter(
            'topic_rc_read').get_parameter_value().string_value
        self.topic_drive_write = self.get_parameter(
            'topic_drive_write').get_parameter_value().string_value
        self.topic_steer_read = self.get_parameter(
            'topic_steer_read').get_parameter_value().string_value
        self.topic_steer_write = self.get_parameter(
            'topic_steer_write').get_parameter_value().string_value
        self.topic_yaw_read = self.get_parameter(
            'topic_yaw_read').get_parameter_value().string_value
        self.topic_referee_constraints = self.get_parameter(
            'topic_referee_constraints').get_parameter_value().string_value

        self.wheel_track = self.get_parameter(
            'wheel_track').get_parameter_value().double_value
        self.wheel_base = self.get_parameter(
            'wheel_base').get_parameter_value().double_value

        self.ecd_zeros = [
            self.get_parameter(
                'offset_fl').get_parameter_value().integer_value,
            self.get_parameter(
                'offset_fr').get_parameter_value().integer_value,
            self.get_parameter(
                'offset_bl').get_parameter_value().integer_value,
            self.get_parameter('offset_br').get_parameter_value().integer_value
        ]

    def cb_rc(self, msg: ReadDJIRC) -> None:
        """
        遥控器数据回调
        
        Args:
            msg: 遥控器数据消息
        """
        self.rc_data = msg
        self.rc_connected = (msg.online == 1)

    def cb_steer_feedback(self, msg: ReadDJIMotor) -> None:
        """
        舵向电机反馈回调
        
        更新当前舵向电机编码器值。注意电机 ID 映射关系。
        
        Args:
            msg: DJI 电机反馈消息
        """
        self.current_steer_ecds[0] = msg.motor1_ecd
        self.current_steer_ecds[1] = msg.motor4_ecd
        self.current_steer_ecds[2] = msg.motor3_ecd
        self.current_steer_ecds[3] = msg.motor2_ecd

    def cb_yaw_feedback(self, msg: ReadLkMotor) -> None:
        """
        Yaw 电机反馈回调
        
        读取 Yaw 电机编码器，计算底盘与云台的夹角。
        编码器 0-65535 映射到 0-2PI。
        
        Args:
            msg: LK 电机反馈消息
        """
        self.gimbal_yaw_angle = (msg.encoder / 65535.0) * 2 * math.pi

    def cb_referee_constraints(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 6:
            return
        self.referee_heat = float(data[0])
        self.referee_heat_limit = float(data[1])
        self.referee_power = float(data[2])
        self.referee_power_limit = float(data[3])
        self.referee_fire_allowed = bool(data[4] > 0.5)
        self.referee_speed_scale = float(data[5])
        self.referee_last_time = self.get_clock().now().nanoseconds / 1e9

    def control_loop(self) -> None:
        """
        主控制循环（500Hz）
        
        执行底盘运动控制，包括：
        1. 安全检查（遥控器连接、急停）
        2. 模式切换（小陀螺模式）
        3. 速度分档（键盘 Shift/Ctrl）
        4. 遥控器和键盘输入映射
        5. 坐标系转换（云台系 -> 底盘系）
        6. 小陀螺速度计算
        7. 运动学解算
        8. 指令发布
        """
        # 安全检查
        if not self.rc_connected or self.rc_data is None:
            self.stop_motors()
            return

        sw_left = self.rc_data.left_switch

        # 1. 死区/急停 (下档位)
        if sw_left == 2:
            self.stop_motors()
            return

        # 2. 模式切换 (上档位) - 小陀螺
        if sw_left == 1 and self.last_switch_state != 1:
            self.mode_rotate_enabled = not self.mode_rotate_enabled
            self.get_logger().info(
                f"Spin Mode Changed: {self.mode_rotate_enabled}")

        # 2.1 键盘模式切换 (V 边沿)
        if self.rc_data.v == 1 and self.last_v_pressed == 0:
            self.mode_rotate_enabled = not self.mode_rotate_enabled
            self.get_logger().info(
                f"Spin Mode Changed By Keyboard: {self.mode_rotate_enabled}")
        self.last_v_pressed = self.rc_data.v

        self.last_switch_state = sw_left

        # 3. 键盘速度分档（legacy: Shift加速, Ctrl减速）
        if self.rc_data.shift == 1:
            self.current_spd_mode = min(8000.0, self.current_spd_mode + 6.0)
        elif self.rc_data.ctrl == 1:
            self.current_spd_mode = max(0.0, self.current_spd_mode - 6.0)

        # 3.1 小陀螺速度调节（legacy: C增, Z减）
        if self.rc_data.c == 1:
            self.spin_spd = min(6500.0, self.spin_spd + 4.0)
        elif self.rc_data.z == 1:
            self.spin_spd = max(0.0, self.spin_spd - 4.0)

        # 4. 遥控器 + 键盘输入 (云台坐标系)
        key_fb = float(self.rc_data.w) - float(self.rc_data.s)
        key_lr = float(self.rc_data.d) - float(self.rc_data.a)

        v_x_gimbal = self.rc_data.right_y * 8000.0 + key_fb * self.current_spd_mode
        v_y_gimbal = self.rc_data.right_x * 8000.0 + key_lr * self.current_spd_mode

        # 5. 坐标系转换 (云台系 -> 底盘系)
        # 将云台视角的平移指令投影到底盘坐标系
        theta = self.gimbal_yaw_angle
        v_x_chassis = v_x_gimbal * \
            math.cos(theta) + v_y_gimbal * math.sin(theta)
        v_y_chassis = -v_x_gimbal * \
            math.sin(theta) + v_y_gimbal * math.cos(theta)

        # 6. 旋转控制 w_z (仅小陀螺模式有效，且受平移速度抑制)
        w_z = 0.0
        if self.mode_rotate_enabled:
            base_spin = self.spin_spd

            # 计算当前平移速度比例 k (基于底盘系速度模长)
            current_speed = math.sqrt(v_x_chassis**2 + v_y_chassis**2)
            k = current_speed / 8000.0  # 归一化

            # 限幅系数 (参考 C++ 逻辑)
            if k > 0.4:
                k = 0.4
            if k < 0.2:
                k = 0.2

            # 减速公式: 速度越快，自旋越慢
            w_z = base_spin * (1.0 - k)
        else:
            w_z = 0.0

        # 7. 死区
        if abs(v_x_gimbal) < 100 and abs(v_y_gimbal) < 100 and abs(w_z) < 100:
            self.stop_motors()
            return

        # 7.1 裁判系统功率约束限幅
        referee_timeout = self.get_parameter(
            'referee_timeout_s').get_parameter_value().double_value
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (now_sec - self.referee_last_time) < referee_timeout:
            speed_scale = max(0.0, min(1.0, self.referee_speed_scale))
            v_x_chassis *= speed_scale
            v_y_chassis *= speed_scale
            w_z *= speed_scale

        # 8. 运动学解算
        drive_speeds, steer_angles = self.kinematics.calculate_motion(
            v_x_chassis, v_y_chassis, w_z,
            self.current_steer_ecds,
            self.ecd_zeros
        )

        self.publish_commands(drive_speeds, steer_angles)

    def publish_commands(self, speeds: list[float], angles: list[int]) -> None:
        """
        发布驱动和舵向指令
        
        注意电机 ID 映射关系：
        - Steer: motor1=FL, motor2=BR, motor3=BL, motor4=FR
        - Drive: motor1=FL, motor2=BR, motor3=BL, motor4=FR
        
        Args:
            speeds: 四个轮子的驱动速度 [FL, FR, BL, BR]
            angles: 四个轮子的舵向目标编码器值 [FL, FR, BL, BR]
        """
        # App3: Steer
        steer_msg = WriteDJIMotor()
        steer_msg.motor1_enable = 1
        steer_msg.motor1_cmd = int(angles[0])  # FL
        steer_msg.motor2_enable = 1
        steer_msg.motor2_cmd = int(angles[1])  # BR (Index 1 -> Motor 2)
        steer_msg.motor3_enable = 1
        steer_msg.motor3_cmd = int(angles[2])  # BL
        steer_msg.motor4_enable = 1
        steer_msg.motor4_cmd = int(angles[3])  # FR (Index 3 -> Motor 4)
        self.pub_steer.publish(steer_msg)

        # App2: Drive
        drive_msg = WriteDJIMotor()
        drive_msg.motor1_enable = 1
        drive_msg.motor1_cmd = int(speeds[0])  # FL
        drive_msg.motor2_enable = 1
        drive_msg.motor2_cmd = int(speeds[1])  # BR
        drive_msg.motor3_enable = 1
        drive_msg.motor3_cmd = int(speeds[2])  # BL
        drive_msg.motor4_enable = 1
        drive_msg.motor4_cmd = int(speeds[3])  # FR
        self.pub_drive.publish(drive_msg)

    def stop_motors(self) -> None:
        """
        停止所有电机
        
        发布零速度指令到驱动电机。
        """
        zero_msg = WriteDJIMotor()
        self.pub_drive.publish(zero_msg)


def main(args=None) -> None:
    """
    主函数入口
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = ChassisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
