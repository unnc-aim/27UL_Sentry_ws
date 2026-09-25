from pathlib import Path
import json
import math
import sqlite3
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag=Path('/home/soyo/rosbag2_2026_09_25-12_05_09/rosbag2_2026_09_25-12_05_09_0.db3')
con=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
all_topics={r[0]:(r[1],r[2]) for r in con.execute('SELECT id,name,type FROM topics')}
names={'/cmd_vel','/cmd_vel_nav2_result','/cmd_vel_controller','/odometry','/amcl_pose','/tf','/tf_static',
       '/chassis_command','/universal_controller/input/ndj','/ecat/sn4653115/app1/read',
       '/ecat/sn4128829/app1/read','/local_plan','/plan','/rosout','/behavior_tree_log'}
topics={i:v for i,v in all_topics.items() if v[0] in names}
classes={i:get_message(v[1]) for i,v in topics.items()}
origin=con.execute('SELECT min(timestamp) FROM messages').fetchone()[0]
rows=defaultdict(list);last=defaultdict(lambda:-math.inf)

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(h):return h.stamp.sec+h.stamp.nanosec*1e-9-origin*1e-9
query='SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ('+','.join('?' for _ in topics)+') ORDER BY timestamp'
for key,time,data in con.execute(query,tuple(topics)):
    name,kind=topics[key];t=(time-origin)*1e-9
    if name in ('/ecat/sn4653115/app1/read','/ecat/sn4128829/app1/read','/chassis_command'):
        if t-last[name]<.02:continue
        last[name]=t
    m=deserialize_message(data,classes[key]);r={'t':t}
    if hasattr(m,'header'):r.update(stamp=stamp(m.header),frame=m.header.frame_id)
    if kind in ('nav_msgs/msg/Odometry','geometry_msgs/msg/PoseWithCovarianceStamped'):
        p=m.pose.pose;r['p']=[p.position.x,p.position.y,p.position.z,yaw(p.orientation)]
        if name=='/amcl_pose':r['cov']=[m.pose.covariance[i] for i in (0,7,35)]
    elif kind in ('geometry_msgs/msg/Twist','geometry_msgs/msg/TwistStamped'):
        v=m.twist if hasattr(m,'twist') else m;r['v']=[v.linear.x,v.linear.y,v.angular.z]
    elif kind=='tf2_msgs/msg/TFMessage':
        r['transforms']=[{'parent':v.header.frame_id,'child':v.child_frame_id,'stamp':stamp(v.header),
           'p':[v.transform.translation.x,v.transform.translation.y,v.transform.translation.z,yaw(v.transform.rotation)]} for v in m.transforms]
    elif kind=='universal_controller/msg/UnifiedInput':
        r.update({k:getattr(m,k) for k in ('navigation_enabled','spin_mode','emergency_stop','autoaim_enabled','friction_on','fire_trigger','vx','vy','wz')})
    elif kind=='custom_msgs/msg/ReadDJIRC':
        r.update({k:getattr(m,k) for k in ('left_switch','right_switch','right_x','right_y','left_x','left_y','dial')})
    elif kind=='custom_msgs/msg/ReadLkMotor':r['encoder']=int(m.encoder)
    elif kind=='chassis_controllers/msg/ChassisControl':r.update(v=[m.x_speed,m.y_speed,m.spin_speed],emergency_stop=int(m.emergency_stop))
    elif kind=='nav_msgs/msg/Path':r.update(count=len(m.poses),endpoint=[m.poses[-1].pose.position.x,m.poses[-1].pose.position.y] if m.poses else [])
    elif kind=='rcl_interfaces/msg/Log':r.update(node=m.name,message=m.msg)
    elif kind=='nav2_msgs/msg/BehaviorTreeLog':
        r['events']=[{'node':v.node_name,'previous':v.previous_status,'current':v.current_status} for v in m.event_log]
    rows[name].append(r)
output=Path(__file__).parent/'topics.json'
output.write_text(json.dumps({'origin_ns':origin,'topics':rows},separators=(',',':')))
print('Saved',output)
print({name:len(data) for name,data in rows.items()})
