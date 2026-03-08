import math

class PID:
    def __init__(self, kp, ki, kd, max_out, max_iout):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_iout = max_iout
        
        self.integral = 0.0
        self.last_err = 0.0
        self.last_d_out = 0.0

    def update(self, error, d_input=None, dt=None, alpha=0.01):
        """
        计算 PID 输出
        :param error: 当前误差
        :param d_input: (可选) 直接用于 D 项计算的输入值。
                        如果提供，D项 = kd * d_input。
                        如果不提供，D项 = kd * (error - last_err)。
        :param dt: (可选) 时间间隔，用于积分和微分的精确计算。
                   如果为None，则假设已经在参数中归一化，或简单累加。
                   通常在高频控制中，KI/KD 参数已经隐含了 dt，这里为了兼容简单实现暂不强制使用 dt。
        :param alpha: D项低通滤波系数，范围 [0, 1]。值越大，滤波越强。
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

    def reset(self):
        self.integral = 0.0
        self.last_err = 0.0
        self.last_d_out = 0.0

    def update_params(self, kp=None, ki=None, kd=None, max_out=None, max_iout=None):
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd
        if max_out is not None: self.max_out = max_out
        if max_iout is not None: self.max_iout = max_iout