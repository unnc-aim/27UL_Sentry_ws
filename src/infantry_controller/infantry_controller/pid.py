"""
PID 控制器模块

本模块提供了一个通用的 PID 控制器实现，支持积分限幅、输出限幅和 D 项低通滤波。

Classes:
    PID: PID 控制器类
"""

class PID:
    """
    PID 控制器类
    
    实现了比例-积分-微分控制算法，支持积分限幅、输出限幅和 D 项低通滤波。
    
    Attributes:
        kp (float): 比例增益
        ki (float): 积分增益
        kd (float): 微分增益
        max_out (float): 最大输出限幅
        max_iout (float): 最大积分限幅
        integral (float): 积分累积值
        last_err (float): 上一次误差值
        last_d_out (float): 上一次 D 项输出值（用于滤波）
    """
    
    def __init__(self, kp: float, ki: float, kd: float, max_out: float, max_iout: float) -> None:
        """
        初始化 PID 控制器
        
        Args:
            kp: 比例增益
            ki: 积分增益
            kd: 微分增益
            max_out: 最大输出限幅
            max_iout: 最大积分限幅
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_iout = max_iout
        
        self.integral = 0.0
        self.last_err = 0.0
        self.last_d_out = 0.0

    def update(self, error: float, d_input: float | None = None, dt: float | None = None, alpha: float = 0.01) -> float:
        """
        计算 PID 输出
        
        Args:
            error: 当前误差
            d_input: (可选) 直接用于 D 项计算的输入值。
                    如果提供，D项 = kd * d_input。
                    如果不提供，D项 = kd * (error - last_err)。
            dt: (可选) 时间间隔，用于积分和微分的精确计算。
               如果为None，则假设已经在参数中归一化，或简单累加。
               通常在高频控制中，KI/KD 参数已经隐含了 dt，这里为了兼容简单实现暂不强制使用 dt。
            alpha: D项低通滤波系数，范围 [0, 1]。值越大，滤波越强。
        
        Returns:
            float: PID 控制器输出值
        """
        # P term
        p_out = self.kp * error
        
        # I term
        self.integral += error
        # 积分限幅
        if self.max_iout > 0:
            self.integral = max(min(self.integral, self.max_iout), -self.max_iout)
        i_out = self.ki * self.integral
        
        # D term
        if d_input is not None:
            # 使用自定义的 D 输入（例如：期望速度 - 测量速度）
            raw_d = self.kd * d_input 
        else:
            # 标准微分（误差差分）
            raw_d = self.kd * (error - self.last_err)
            
        # D项低通滤波，减小高频噪声引起的电机异响抖动
        d_out = alpha * raw_d + (1.0 - alpha) * self.last_d_out
        self.last_d_out = d_out
        
        self.last_err = error
        
        output = p_out + i_out + d_out
        
        # 总输出限幅
        if self.max_out > 0:
            output = max(min(output, self.max_out), -self.max_out)
            
        return output

    def reset(self) -> None:
        """
        重置 PID 控制器状态
        
        清除积分累积值、上一次误差值和 D 项滤波值。
        """
        self.integral = 0.0
        self.last_err = 0.0
        self.last_d_out = 0.0

    def update_params(self, kp: float | None = None, ki: float | None = None, kd: float | None = None, 
                     max_out: float | None = None, max_iout: float | None = None) -> None:
        """
        更新 PID 参数
        
        Args:
            kp: 新的比例增益（可选）
            ki: 新的积分增益（可选）
            kd: 新的微分增益（可选）
            max_out: 新的最大输出限幅（可选）
            max_iout: 新的最大积分限幅（可选）
        """
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd
        if max_out is not None: self.max_out = max_out
        if max_iout is not None: self.max_iout = max_iout
