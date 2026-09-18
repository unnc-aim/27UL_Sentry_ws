"""Synthetic stationary-body lever arm, including a wrap through pi."""
import math
from calibrate_lidar_offset import fit
rows=[]
for i in range(100):
    angle=2.5+i*.025;c=math.cos(angle);s=math.sin(angle)
    rows.append({'gimbal_yaw':math.atan2(s,c),'position':[1+c*.025-s*.06,2+s*.025+c*.06],'chassis':[0,0,0]})
r=fit(rows)
assert math.dist(r['correction_xy_m'],[.025,.06])<1e-10
assert r['fit_rms_m']<1e-10
rows[0]['chassis'][0]=.1
try:fit(rows)
except ValueError:pass
else:raise AssertionError('Moving-body data must be rejected')
print('PASS: lever-arm recovery across angle wrap and rejection of commanded motion')
