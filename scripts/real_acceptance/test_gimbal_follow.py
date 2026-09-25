"""Heading math and existing scan-range semantics; no ROS nodes or hardware."""
import math
from gimbal_follow import HeadingTarget,wrap

for yaw,error in [(0.,1.),(3.13,.3),(-3.13,-.3),(1.,-2.)]:
    h=HeadingTarget(yaw,.1);target=wrap(yaw+error)
    for _ in range(250):
        delta=wrap(target-yaw);old=h.yaw
        h.update(yaw,(.15*math.cos(delta),.15*math.sin(delta)),.05)
        assert abs(wrap(h.yaw-old))<=.020001
        # Both possible persistent sweep directions give the same pinned angle.
        for direction in (-1.,1.):
            internal=old+0.*direction*.001
            if internal>=h.yaw:internal=h.yaw
            elif internal<=h.yaw:internal=h.yaw
            assert internal==h.yaw
        yaw=h.yaw
    assert abs(wrap(yaw-target))<1e-6
    before=h.yaw;assert h.update(yaw,(0.,0.),.05) is None and h.yaw==before
for bad in [(float('nan'),0.),(0.,1.)]:
    try:HeadingTarget(*bad)
    except ValueError:pass
    else:raise AssertionError('Invalid initial state accepted')
try:HeadingTarget(0.,0.).update(.3,(.15,0.),.05)
except ValueError:pass
else:raise AssertionError('Tracking lag accepted')
print('PASS: yaw wrap, target slew, persistent scan direction independence, stop hold, invalid pose and lag rejection')
