import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from custom_msgs.msg import ReadDJIMotor, WriteDJIMotor, ReadDJIRC, ReadLkMotor
from infantry_controller.chassis_kinematics import SwerveKinematics

class ChassisController(Node):
    def __init__(self):
        super().__init__('chassis_controller')

        # ================= 参数声明与加载 =================
        self._declare_and_load_params()

        # ================= 运动学初始化 =================
        # 静态加载，无动态回调，确保实时性
        self.kinematics = SwerveKinematics(
            wheel_track=self.wheel_track, 
            wheel_base=self.wheel_base
        )
        self.get_logger().info(f"Geometry Loaded: Track={self.wheel_track}m, Base={self.wheel_base}m")

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
        self.mode_rotate_enabled = False # 小陀螺模式标志
        self.last_switch_state = 0       # 拨杆边沿检测
        
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

    def _declare_and_load_params(self):
        """加载YAML参数 - 静态加载"""
        # --- Topic ---
        self.declare_parameter('topic_rc_read', '/ecat/sn4587585/app1/read')
        self.declare_parameter('topic_drive_write', '/ecat/sn4587586/app1/write')
        self.declare_parameter('topic_steer_read', '/ecat/sn4587586/app2/read')
        self.declare_parameter('topic_steer_write', '/ecat/sn4587586/app2/write')
        self.declare_parameter('topic_yaw_read', '/ecat/sn4587586/app3/read')
        
        # --- Geometry ---
        self.declare_parameter('wheel_track', 0.4)
        self.declare_parameter('wheel_base', 0.4)

        # --- Offsets ---
        self.declare_parameter('offset_fl', 0)
        self.declare_parameter('offset_fr', 0)
        self.declare_parameter('offset_bl', 0)
        self.declare_parameter('offset_br', 0)

        # --- 读取参数值 ---
        self.topic_rc_read = self.get_parameter('topic_rc_read').value
        self.topic_drive_write = self.get_parameter('topic_drive_write').value
        self.topic_steer_read = self.get_parameter('topic_steer_read').value
        self.topic_steer_write = self.get_parameter('topic_steer_write').value
        self.topic_yaw_read = self.get_parameter('topic_yaw_read').value
        
        self.wheel_track = self.get_parameter('wheel_track').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        self.ecd_zeros = [
            self.get_parameter('offset_fl').value,
            self.get_parameter('offset_fr').value,
            self.get_parameter('offset_bl').value,
            self.get_parameter('offset_br').value
        ]

    def cb_rc(self, msg):
        self.rc_data = msg
        self.rc_connected = (msg.online == 1)

    def cb_steer_feedback(self, msg):
        """舵向电机反馈映射"""
        self.current_steer_ecds[0] = msg.motor1_ecd
        self.current_steer_ecds[1] = msg.motor4_ecd 
        self.current_steer_ecds[2] = msg.motor3_ecd
        self.current_steer_ecds[3] = msg.motor2_ecd 

    def cb_yaw_feedback(self, msg: ReadLkMotor):
        """
        读取 Yaw 电机编码器，计算底盘与云台的夹角
        编码器 0-65535 映射到 0-2PI
        """
        self.gimbal_yaw_angle = (msg.encoder / 65535.0) * 2 * math.pi

    def control_loop(self):
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
            self.get_logger().info(f"Spin Mode Changed: {self.mode_rotate_enabled}")
        
        self.last_switch_state = sw_left

        # 3. 遥控器输入 (云台坐标系)
        v_x_gimbal = self.rc_data.right_y * 8000.0  
        v_y_gimbal = self.rc_data.right_x * 8000.0  
        
        # 4. 坐标系转换 (云台系 -> 底盘系)
        # 将云台视角的平移指令投影到底盘坐标系
        theta = self.gimbal_yaw_angle
        v_x_chassis = v_x_gimbal * math.cos(theta) + v_y_gimbal * math.sin(theta)
        v_y_chassis = -v_x_gimbal * math.sin(theta) + v_y_gimbal * math.cos(theta)

        # 5. 旋转控制 w_z (仅小陀螺模式有效，且受平移速度抑制)
        w_z = 0.0
        if self.mode_rotate_enabled:
            base_spin = 3000.0
            
            # 计算当前平移速度比例 k (基于底盘系速度模长)
            current_speed = math.sqrt(v_x_chassis**2 + v_y_chassis**2)
            k = current_speed / 8000.0 # 归一化
            
            # 限幅系数 (参考 C++ 逻辑)
            if k > 0.4: k = 0.4
            if k < 0.2: k = 0.2
            
            # 减速公式: 速度越快，自旋越慢
            w_z = base_spin * (1.0 - k)
        else:
            w_z = 0.0

        # 6. 摇杆死区
        if abs(v_x_gimbal) < 100 and abs(v_y_gimbal) < 100 and abs(w_z) < 100:
            self.stop_motors()
            return

        # 7. 运动学解算
        drive_speeds, steer_angles = self.kinematics.calculate_motion(
            v_x_chassis, v_y_chassis, w_z, 
            self.current_steer_ecds, 
            self.ecd_zeros
        )

        self.publish_commands(drive_speeds, steer_angles)

    def publish_commands(self, speeds, angles):
        """发布指令，注意电机 ID 映射"""
        # App3: Steer
        steer_msg = WriteDJIMotor()
        steer_msg.motor1_enable = 1; steer_msg.motor1_cmd = int(angles[0]) # FL
        steer_msg.motor2_enable = 1; steer_msg.motor2_cmd = int(angles[1]) # BR (Index 1 -> Motor 2)
        steer_msg.motor3_enable = 1; steer_msg.motor3_cmd = int(angles[2]) # BL
        steer_msg.motor4_enable = 1; steer_msg.motor4_cmd = int(angles[3]) # FR (Index 3 -> Motor 4)
        self.pub_steer.publish(steer_msg)

        # App2: Drive
        drive_msg = WriteDJIMotor()
        drive_msg.motor1_enable = 1; drive_msg.motor1_cmd = int(speeds[0]) # FL
        drive_msg.motor2_enable = 1; drive_msg.motor2_cmd = int(speeds[1]) # BR
        drive_msg.motor3_enable = 1; drive_msg.motor3_cmd = int(speeds[2]) # BL
        drive_msg.motor4_enable = 1; drive_msg.motor4_cmd = int(speeds[3]) # FR
        self.pub_drive.publish(drive_msg)

    def stop_motors(self):
        zero_msg = WriteDJIMotor()
        self.pub_drive.publish(zero_msg)

def main(args=None):
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
