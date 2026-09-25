"""Rate-limited heading target for the existing Hub scan-range interface."""
import math


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


class HeadingTarget:
    def __init__(self,yaw,pitch):
        if not all(math.isfinite(v) for v in (yaw,pitch)) or not math.radians(-25)<=pitch<=math.radians(30):
            raise ValueError('Invalid initial gimbal pose')
        self.yaw=yaw;self.pitch=pitch

    def update(self,imu_yaw,velocity,dt):
        if not all(math.isfinite(v) for v in (imu_yaw,*velocity,dt)) or dt<=0:
            raise ValueError('Invalid heading feedback')
        if abs(wrap(self.yaw-imu_yaw))>.15:raise ValueError('Gimbal target tracking lag')
        if math.hypot(*velocity)<.03:return None
        error=math.atan2(velocity[1],velocity[0])
        delta=wrap(imu_yaw+error-self.yaw)
        limit=.4*min(dt,.1)
        self.yaw=wrap(self.yaw+max(-limit,min(limit,delta)))
        return error

    def message(self,stamp):
        from pb_rm_interfaces.msg import GimbalCmd
        msg=GimbalCmd();msg.header.stamp=stamp
        msg.yaw_type=msg.pitch_type=GimbalCmd.VELOCITY
        # Zero scan velocity + coincident limits pins the existing controller's
        # target, independent of its persistent sweep direction. No free sweep.
        msg.velocity.yaw=msg.velocity.pitch=0.
        msg.velocity.yaw_min_range=msg.velocity.yaw_max_range=self.yaw
        msg.velocity.pitch_min_range=msg.velocity.pitch_max_range=self.pitch
        return msg
