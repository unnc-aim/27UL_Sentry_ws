#!/usr/bin/env python3
"""Bounded real navigation trial using isolated Nav2 and the existing Hub input.

No RC, emergency-stop, motor, fire or gimbal control interfaces are modified.
Only runs after --execute; real motion is gated by fresh telemetry and RC mode.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import time
import yaml

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from universal_controller.msg import UnifiedInput
from custom_msgs.msg import ReadDJIRC, ReadLkMotorMulti
from chassis_controllers.msg import ChassisControl


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--course', choices=['outback','rectangle'], default='outback')
    parser.add_argument('--landmarks', type=Path)
    parser.add_argument('--segmented-global',action='store_true')
    args=parser.parse_args()
    if not args.execute:parser.error('--execute is required')
    live,test=Context(),Context()
    os.environ['ROS_LOCALHOST_ONLY']='0';live.init(domain_id=0)
    os.environ['ROS_LOCALHOST_ONLY']='1';test.init(domain_id=88)
    hardware=rclpy.create_node('acceptance_navigation_gate',context=live,enable_rosout=False,start_parameter_services=False)
    nav=rclpy.create_node('acceptance_navigation_trial',context=test,enable_rosout=False,start_parameter_services=False)
    exlive=SingleThreadedExecutor(context=live);exnav=SingleThreadedExecutor(context=test)
    exlive.add_node(hardware);exnav.add_node(nav)
    buffer=Buffer(node=nav);listener=TransformListener(buffer,nav)
    state={}
    def receive(key,msg):state[key]=(time.monotonic(),msg)
    for node,key,topic,cls in [
        (hardware,'rc','/universal_controller/input/ndj',UnifiedInput),
        (hardware,'raw','/ecat/sn4653115/app1/read',ReadDJIRC),
        (hardware,'chassis','/chassis_command',ChassisControl),
        (hardware,'drive','/ecat/sn2228292/app1/read',ReadLkMotorMulti),
        (hardware,'steer','/ecat/sn2228292/app2/read',ReadLkMotorMulti),
        (nav,'scan','/obstacle_scan',LaserScan),
        (nav,'odom','/odometry',Odometry),
        (nav,'joints','/joint_states',JointState),
        (nav,'command','/static_acceptance/cmd_vel',TwistStamped),
    ]:
        node.create_subscription(cls,topic,lambda msg,k=key:receive(k,msg),qos_profile_sensor_data)
    if args.segmented_global:
        nav.create_subscription(Bool,'/acceptance/localization_ready',lambda msg:receive('localization_ready',msg),1)
    client=ActionClient(nav,NavigateToPose,'/navigate_to_pose')
    publisher=None;active=None;origin=None
    report={'radius_m':.2,'max_speed_mps':.15,'physical_acceptance_passed':False,
            'hub_input_linear_sign':-1,'waypoints':[],'trace':[],'scope':'Current local map; no prebuilt-field global localization certification'}
    output=Path(os.environ.get('SENTRY_ACCEPTANCE_OUTPUT', str(Path(__file__).parent/'results/current')))/'navigation_trial.json'
    def spin():exlive.spin_once(timeout_sec=.002);exnav.spin_once(timeout_sec=.002)
    def position():
        tf=buffer.lookup_transform('map','base_footprint',Time())
        stamp=tf.header.stamp
        age=nav.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
        if not -.05<=age<.3:raise RuntimeError('Pose TF stale')
        return tf.transform.translation.x,tf.transform.translation.y
    def guard():
        if args.segmented_global and ('localization_ready' not in state or time.monotonic()-state['localization_ready'][0]>1.2 or not state['localization_ready'][1].data):
            raise RuntimeError('Global localization readiness lost')
        for key in ('rc','raw','chassis','drive','steer','scan','odom','joints'):
            if key not in state or time.monotonic()-state[key][0]>.25:raise RuntimeError(key+' stale/missing')
        rc=state['rc'][1];raw=state['raw'][1]
        if not rc.connected or rc.emergency_stop or not rc.navigation_enabled:raise RuntimeError('RC authorization revoked')
        if not raw.online or any(abs(v)>.05 for v in (raw.left_x,raw.left_y,raw.right_x,raw.right_y,raw.dial)):raise RuntimeError('RC manual takeover')
        if rc.fire_trigger or rc.friction_on or abs(rc.wz)>.001:raise RuntimeError('Non-navigation actuator request')
        if state['chassis'][1].emergency_stop:raise RuntimeError('Chassis emergency stop')
        if abs(state['chassis'][1].spin_speed)>.01:raise RuntimeError('Unexpected chassis spin')
        for key in ('drive','steer'):
            if not all(getattr(state[key][1],f'motor{i}_online') for i in range(1,5)):raise RuntimeError(key+' motor offline')
        for key in ('odom','scan','joints'):
            stamp=state[key][1].header.stamp
            age=nav.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
            if not -.05<=age<.3:raise RuntimeError(key+' timestamp stale')
        scan=state['scan'][1]
        if scan.header.frame_id!='base_footprint':raise RuntimeError('Unexpected scan frame')
        ranges=[r for r in scan.ranges if math.isfinite(r) and scan.range_min<=r<=scan.range_max]
        if len(ranges)<100 or min(ranges)<.4:raise RuntimeError('Clearance/scan coverage gate')
        if len(hardware.get_publishers_info_by_topic('/cmd_vel'))>(1 if publisher else 0):raise RuntimeError('Competing velocity publisher')
        p=position()
        if origin and math.dist(p,origin)>.35:raise RuntimeError('Trial displacement bound exceeded')
        return p
    def send(vx=0.,vy=0.):
        msg=TwistStamped();msg.header.stamp=hardware.get_clock().now().to_msg();msg.header.frame_id='gimbal_yaw'
        # Existing Hub negates both navigation axes. Match its input contract
        # without changing RC arbitration; confirmed by two-axis real pulses.
        msg.twist.linear.x=-vx;msg.twist.linear.y=-vy;publisher.publish(msg)
    def stop(seconds=2.):
        end=time.monotonic()+seconds;next_send=0.
        while time.monotonic()<end:
            spin()
            if time.monotonic()>=next_send:send();next_send=time.monotonic()+.05
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:spin()
        origin=guard()
        frames=yaml.safe_load(buffer.all_frames_as_yaml())
        if frames.get('front_mid360',{}).get('parent')!='gimbal_yaw':
            raise RuntimeError('Acceptance requires confirmed gimbal-mounted lidar model')
        if not client.wait_for_server(timeout_sec=2):raise RuntimeError('NavigateToPose unavailable')
        tf=buffer.lookup_transform('map','base_footprint',Time());q=tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        publisher=hardware.create_publisher(TwistStamped,'/cmd_vel',10)
        # Automatic local-map targets; no hand-entered world coordinates.
        offsets=[('forward',.20,0),('home',0,0)] if args.course=='outback' else [('forward',.20,0),('corner',.20,.20),('left',0,.20),('home',0,0)]
        targets=[(name,origin[0]+dx*math.cos(yaw)-dy*math.sin(yaw),origin[1]+dx*math.sin(yaw)+dy*math.cos(yaw)) for name,dx,dy in offsets]
        if args.landmarks:
            saved=json.loads(args.landmarks.read_text())
            if saved['frame']!='map':raise RuntimeError('Landmark frame must be map')
            targets=[(t['name'],float(t['x']),float(t['y'])) for t in saved['targets']]
            report['landmarks']=str(args.landmarks)
        report['course']='saved_landmarks' if args.landmarks else args.course
        for name,gx,gy in targets:
            guard()
            if args.segmented_global:origin=position()
            stop(.25)
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=nav.get_clock().now().to_msg()
            goal.pose.pose.position.x=gx;goal.pose.pose.position.y=gy;goal.pose.pose.orientation.w=1.
            goal.behavior_tree=str(Path(__file__).resolve().parents[1]/'coordinate_sim/navigate.xml')
            future=client.send_goal_async(goal);deadline=time.monotonic()+5
            while not future.done() and time.monotonic()<deadline:spin();guard();send()
            if not future.done():raise RuntimeError('Goal acceptance timeout')
            active=future.result()
            if not active.accepted:raise RuntimeError('Navigation goal rejected')
            result=active.get_result_async();deadline=time.monotonic()+30;next_send=0.;best=math.dist(position(),(gx,gy));progress=time.monotonic()
            while not result.done():
                spin();p=guard();distance=math.dist(p,(gx,gy))
                if distance<best-.01:best=distance;progress=time.monotonic()
                if distance> .30:raise RuntimeError('Motion diverged from target')
                if time.monotonic()>deadline or time.monotonic()-progress>8:raise RuntimeError('Navigation progress timeout')
                if time.monotonic()>=next_send:
                    vx=vy=0.
                    if 'command' in state and time.monotonic()-state['command'][0]<.2:
                        cmd=state['command'][1]
                        if abs(cmd.twist.angular.z)>.01:raise RuntimeError('Unexpected Nav2 rotation')
                        vx,vy=cmd.twist.linear.x,cmd.twist.linear.y
                        norm=math.hypot(vx,vy)
                        if not math.isfinite(norm):raise RuntimeError('Non-finite navigation command')
                        if norm>.15:vx*=.15/norm;vy*=.15/norm
                    send(vx,vy);next_send=time.monotonic()+.05
                    report['trace'].append({'time':time.monotonic(),'target':name,'pose':p,'distance':distance,'command':[vx,vy],'hub_command':[-vx,-vy]})
            status=result.result().status;active=None;stop()
            error=math.dist(position(),(gx,gy))
            report['waypoints'].append({'name':name,'target':[gx,gy],'status':status,'position_error_m':error})
            if status!=4 or error>.06:raise RuntimeError('Goal did not meet measured tolerance')
        report['local_navigation_passed']=True
    except Exception as exc:
        report['abort_reason']=str(exc)
    finally:
        if active:
            try:active.cancel_goal_async()
            except Exception:pass
        if publisher:stop()
        report['final_wheel_speeds']=[getattr(state['drive'][1],f'motor{i}_speed') for i in range(1,5)] if 'drive' in state else None
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='trace'}))
        exlive.shutdown();exnav.shutdown();hardware.destroy_node();nav.destroy_node();live.try_shutdown();test.try_shutdown()
    return 0 if report.get('local_navigation_passed') else 1


if __name__=='__main__':raise SystemExit(main())
