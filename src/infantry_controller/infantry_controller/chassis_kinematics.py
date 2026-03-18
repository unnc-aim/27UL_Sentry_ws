"""
舵轮底盘运动学核心解算模块

本模块提供舵轮底盘的运动学正解算，将底盘速度指令转换为各轮子的驱动速度和舵向角度。

Classes:
    SwerveKinematics: 舵轮运动学解算器
"""
import math

class SwerveKinematics:
    """
    舵轮底盘运动学解算器
    
    实现四轮舵轮底盘的运动学正解算，支持全向移动和旋转。
    
    Attributes:
        geometry_factor (float): 几何中心到轮子的距离系数
        k (float): 归一化比例系数
    """
    
    def __init__(self, wheel_track: float, wheel_base: float) -> None:
        """
        初始化运动学参数
        
        Args:
            wheel_track: 轮距（宽度，单位：米）
            wheel_base: 轴距（长度，单位：米）
        """
        # 计算几何中心到轮子的距离系数
        # 对应原 C++ 代码中的 half_of_sqrt_2 (如果长宽相等)
        self.geometry_factor = math.sqrt(wheel_track**2 + wheel_base**2) / 2.0
        # 归一化比例系数 (假设长宽相等，简化计算)
        self.k = 0.7071068 

    def calculate_motion(self, v_x: float, v_y: float, omega: float, 
                        current_steer_ecds: list[int], ecd_zeros: list[int]) -> tuple[list[float], list[int]]:
        """
        计算四个轮子的速度和舵向角度
        
        Args:
            v_x: 前进速度（m/s 或归一化单位）
            v_y: 横移速度（m/s 或归一化单位）
            omega: 旋转速度（rad/s）
            current_steer_ecds: 当前舵向电机编码器值列表 [FL, FR, BL, BR]
            ecd_zeros: 舵向电机零位偏移列表 [FL, FR, BL, BR]
        
        Returns:
            tuple[list[float], list[int]]: (驱动速度列表, 舵向目标编码器值列表)
                - drive_speeds: 四个轮子的驱动速度 [FL, FR, BL, BR]
                - steer_targets_ecd: 四个轮子的舵向目标编码器值 [FL, FR, BL, BR]
        """
        # 预计算旋转产生的线速度分量 v_w
        v_w = omega 
        
        # 计算每个轮子的速度矢量 (vx_i, vy_i)
        # 顺序: FL(前左), FR(前右), BL(后左), BR(后右)
        # 符号参考原C++代码:
        # FL: (+, +), FR: (+, -), BL: (-, +), BR: (-, -)
        vectors = [
            (v_x + self.k * v_w, v_y + self.k * v_w), # Front Left
            (v_x - self.k * v_w, v_y + self.k * v_w), # Front Right
            (v_x + self.k * v_w, v_y - self.k * v_w), # Back Left
            (v_x - self.k * v_w, v_y - self.k * v_w)  # Back Right
        ]
        
        drive_cmds = []
        steer_cmds = []
        
        for i, (vx_i, vy_i) in enumerate(vectors):
            # 1. 计算目标线速度模长
            speed = math.sqrt(vx_i**2 + vy_i**2)
            
            # 2. 计算目标角度 (atan2 返回 -pi 到 pi)
            angle_rad = math.atan2(vy_i, vx_i)
            
            # 3. 转换为 0-8191 坐标系下的目标编码器值
            if angle_rad < 0:
                angle_rad += 2 * math.pi
                
            raw_target_ecd = int((angle_rad / (2 * math.pi)) * 8192)
            # 加上零位偏移
            target_ecd_with_offset = (raw_target_ecd + ecd_zeros[i]) % 8192
            
            # 4. 最短路径优化 (Shortest Path)
            # 如果转动超过90度，则反转电机速度方向
            current_ecd = current_steer_ecds[i]
            final_ecd, direction_mult = self._calc_shortest_path(current_ecd, target_ecd_with_offset)
            
            # 5. 限幅 (参考 limitMM 8000)
            speed = max(min(speed, 8000), -8000)
            
            drive_cmds.append(speed * direction_mult)
            steer_cmds.append(final_ecd)
            
        return drive_cmds, steer_cmds

    def _calc_shortest_path(self, current_ecd: int, target_ecd: int) -> tuple[int, float]:
        """
        计算编码器最短旋转路径
        
        比较直接转到目标和转到目标对面（同时反转电机）两种方案，选择转动角度最小的方案。
        
        Args:
            current_ecd: 当前编码器值（0-8191）
            target_ecd: 目标编码器值（0-8191）
        
        Returns:
            tuple[int, float]: (最优目标编码器值, 速度方向系数)
                - 最优目标编码器值: 0-8191
                - 速度方向系数: 1.0 或 -1.0
        """
        range_val = 8192
        half_range = 4096
        
        # 方案A: 直接转到目标
        diff_a = (target_ecd - current_ecd + range_val) % range_val
        if diff_a > half_range:
            diff_a -= range_val # 转为 -4096 到 4096
            
        # 方案B: 转到目标对面 (目标+180度)，同时电机反转
        target_flipped = (target_ecd + half_range) % range_val
        diff_b = (target_flipped - current_ecd + range_val) % range_val
        if diff_b > half_range:
            diff_b -= range_val
            
        # 比较绝对距离，选择转动角度最小的方案
        if abs(diff_a) <= abs(diff_b):
            return target_ecd, 1.0
        else:
            return target_flipped, -1.0
