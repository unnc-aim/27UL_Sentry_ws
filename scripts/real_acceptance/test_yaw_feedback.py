"""Geometry regression only; does not initialize ROS or send commands."""
import math
from yaw_feedback import encoder_yaw

zero = 18032
assert encoder_yaw(zero, zero) == 0
assert abs(encoder_yaw(0, zero)-encoder_yaw(65535, zero)) < 1e-4
for encoder in [0, 12000, zero, 30000, 54310, 65535]:
    angle = encoder_yaw(encoder, zero)
    assert -math.pi-1e-4 <= angle <= math.pi+1e-4
    # Existing Hub getter is -angle; its velocity conversion applies R(-getter).
    theta = -angle
    for x, y in [(1., 0.), (0., 1.), (.3, -.2)]:
        hub = (x*math.cos(theta)+y*math.sin(theta), -x*math.sin(theta)+y*math.cos(theta))
        tf = (x*math.cos(angle)-y*math.sin(angle), x*math.sin(angle)+y*math.cos(angle))
        assert math.dist(hub, tf) < 1e-12
print('PASS: encoder zero, wrap and existing Hub/TF rotation convention')

# Complete planar navigation conversion, including the existing Hub sign contract.
# Physical lidar now belongs to gimbal_yaw. Body and gimbal headings can differ.
def rotate(v, angle):
    return (v[0]*math.cos(angle)-v[1]*math.sin(angle),
            v[0]*math.sin(angle)+v[1]*math.cos(angle))
for body in (-2.7, 0., 1.4):
    for joint in (-2.2, .4, 2.8):
        desired=(.07, -.12)
        gimbal_velocity=rotate(desired, -(body+joint))
        bridge=tuple(-v for v in gimbal_velocity)
        hub=rotate(tuple(-v for v in bridge),joint)
        world=rotate(hub,body)
        assert math.dist(world,desired)<1e-12
print('PASS: navigation adapter cancels existing Hub sign across body/gimbal headings')
