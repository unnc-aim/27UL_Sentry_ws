from types import SimpleNamespace as O
from localization_probe import aligned_pair

def msg(sec, nanosec=0):return O(header=O(stamp=O(sec=sec,nanosec=nanosec)))
p1,p2=msg(10),msg(10,200000000)
s1=msg(10)
assert aligned_pair([p1,p2],[s1])==(p1,s1)
s2=msg(10,200000000)
assert aligned_pair([p1,p2],[s1,s2])==(p2,s2)
assert aligned_pair([p2],[msg(9,990000000)]) is None
assert aligned_pair([msg(11)],[msg(10,990000000)]) is not None
assert aligned_pair([p1],[]) is None
print('PASS: newest matched pair, late arrival, dropped frame, second rollover and empty input')

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from localization_probe import pose_status
pose = PoseWithCovarianceStamped()
pose.header.frame_id = 'map'
pose.header.stamp.sec = 20
pose.pose.pose.orientation.w = 1.
scan = LaserScan()
scan.header.frame_id = 'base_footprint'
scan.header.stamp.sec = 20
scan.range_min, scan.range_max, scan.ranges = .3, 10., [1.]
assert pose_status(pose, scan, 20.1, 19.9)[0]
assert not pose_status(pose, scan, 21., 19.9)[0]
assert not pose_status(pose, scan, 20.1, 20.2)[0]
pose.pose.covariance[0] = .04
assert not pose_status(pose, scan, 20.1, 19.9)[0]
pose.pose.covariance[0] = float('nan')
try:
    pose_status(pose, scan, 20.1, 19.9)
    raise AssertionError('Expected finite covariance validation')
except ValueError:
    pass
print('PASS: fresh native poses, initialization time and finite covariance')
