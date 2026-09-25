#!/usr/bin/env python3
"""Bounded real navigation trial using isolated Nav2 and the existing Hub input.

No RC, emergency-stop, motor, fire or gimbal control interfaces are modified.
Only runs after --execute; real motion is gated by fresh telemetry and RC mode.
"""
import argparse
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import time
import signal
import threading
import yaml

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan, JointState, PointCloud2, Imu
from pb_rm_interfaces.msg import GimbalCmd
from gimbal_follow import HeadingTarget
from speed_profile import limited_velocity
from obstacle_hold import ObstacleHold
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from startup_trial import path_is_free
from saved_goal import validate_selected
from universal_controller.msg import UnifiedInput
from custom_msgs.msg import ReadDJIRC, ReadLkMotorMulti
from chassis_controllers.msg import ChassisControl


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--course', choices=['outback','rectangle'], default='outback')
    parser.add_argument('--landmarks', type=Path)
    parser.add_argument('--segmented-global',action='store_true')
    parser.add_argument('--saved-goal',action='store_true',help='One saved-map goal with continuous replanning, no scan or rotation')
    parser.add_argument('--face-motion',action='store_true')
    parser.add_argument('--align-only',action='store_true',help='Heading test with zero chassis velocity')
    parser.add_argument('--max-speed',type=float,default=.15)
    parser.add_argument('--stop-after-travel',type=float,help='Controlled stopping measurement after this displacement')
    args=parser.parse_args()
    if not math.isfinite(args.max_speed) or not 0<args.max_speed<=1.:
        parser.error('--max-speed must be finite and in (0, 1.0] m/s')
    if args.max_speed>.15 and not (args.saved_goal and args.face_motion):
        parser.error('Higher speed requires saved-map navigation and motion-facing lidar')
    if args.stop_after_travel is not None and (not math.isfinite(args.stop_after_travel) or not .2<=args.stop_after_travel<=1.):
        parser.error('--stop-after-travel must be between 0.2 and 1.0 m')
    if not args.execute:parser.error('--execute is required')
    if args.saved_goal and (not args.landmarks or args.segmented_global):
        parser.error('--saved-goal requires --landmarks and excludes --segmented-global')
    if args.face_motion and not args.saved_goal:parser.error('--face-motion requires --saved-goal')
    if args.align_only and not args.face_motion:parser.error('--align-only requires --face-motion')
    live,test=Context(),Context()
    os.environ['ROS_LOCALHOST_ONLY']='0';live.init(domain_id=0)
    os.environ['ROS_LOCALHOST_ONLY']='1';test.init(domain_id=88)
    hardware=rclpy.create_node('acceptance_navigation_gate',context=live,enable_rosout=False,start_parameter_services=False)
    nav=rclpy.create_node('acceptance_navigation_trial',context=test,enable_rosout=False,start_parameter_services=False)
    exlive=SingleThreadedExecutor(context=live);exnav=SingleThreadedExecutor(context=test)
    exlive.add_node(hardware);exnav.add_node(nav)
    buffer=Buffer(node=nav);listener=TransformListener(buffer,nav)
    state={}
    rc_anomalies=deque(maxlen=100)
    def receive(key,msg):
        now=time.monotonic()
        state[key]=(now,msg)
        if key=='raw':
            channels={name:float(getattr(msg,name)) for name in
                      ('left_x','left_y','right_x','right_y','dial')}
            if msg.online!=1 or any(not math.isfinite(v) or abs(v)>.05 for v in channels.values()):
                rc_anomalies.append({'time':now,'online':int(msg.online),'channels':channels})
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
    if args.segmented_global or args.saved_goal:
        nav.create_subscription(Bool,'/acceptance/localization_ready',lambda msg:receive('localization_ready',msg),1)
    if args.saved_goal:
        nav.create_subscription(NavPath,'/plan',lambda msg:receive('plan',msg),10)
        for key in ('terrain_map','terrain_map_ext'):
            nav.create_subscription(PointCloud2,'/'+key,lambda msg,k=key:receive(k,msg),qos_profile_sensor_data)
        for key,topic in [('map','/map'),('local_costmap','/local_costmap/costmap'),('global_costmap','/global_costmap/costmap')]:
            nav.create_subscription(OccupancyGrid,topic,lambda msg,k=key:receive(k,msg),
                                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if args.face_motion:
        hardware.create_subscription(Imu,'/ecat/sn4653115/app2/read',lambda msg:receive('gimbal_imu',msg),qos_profile_sensor_data)
        hardware.create_subscription(Int32,'/auto_aim_switch',lambda msg:receive('aim_switch',msg),qos_profile_sensor_data)
    client=ActionClient(nav,NavigateToPose,'/navigate_to_pose')
    # Drain sensor/RC callbacks continuously. File checkpoints and graph queries
    # in the control loop must not hold up receipt of fresh safety telemetry.
    workers=[threading.Thread(target=executor.spin,daemon=True) for executor in (exlive,exnav)]
    for worker in workers:worker.start()
    publisher=None;active=None;origin=None;gimbal_publisher=None;heading=None;autoaim_gate_publisher=None
    report={'radius_m':.2,'max_speed_mps':args.max_speed,'physical_acceptance_passed':False,
            'hub_input_linear_sign':-1,'waypoints':[],'trace':[],'scope':'Current local map; no prebuilt-field global localization certification'}
    output=Path(os.environ.get('SENTRY_ACCEPTANCE_OUTPUT', str(Path(__file__).parent/'results/current')))/'navigation_trial.json'
    last_checkpoint=0.
    last_sent_speed=0.;last_sent_time=time.monotonic()
    report['status']='running'
    def save_report():
        output.parent.mkdir(parents=True,exist_ok=True)
        temporary=output.with_suffix('.tmp')
        with temporary.open('w') as file:
            json.dump(report,file,indent=2);file.write('\n');file.flush();os.fsync(file.fileno())
        temporary.replace(output)
        directory=os.open(output.parent,os.O_RDONLY)
        try:os.fsync(directory)
        finally:os.close(directory)
    def spin():time.sleep(.004)
    def position():
        tf=buffer.lookup_transform('map','base_footprint',Time())
        stamp=tf.header.stamp
        age=nav.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
        if not -.05<=age<.3:raise RuntimeError('Pose TF stale')
        return tf.transform.translation.x,tf.transform.translation.y
    def guard(allow_localization_wait=False, allow_obstacle_wait=False):
        if (args.segmented_global or args.saved_goal) and ('localization_ready' not in state or time.monotonic()-state['localization_ready'][0]>1.2 or (not allow_localization_wait and not state['localization_ready'][1].data)):
            raise RuntimeError('Global localization readiness lost')
        for key in ('rc','raw','chassis','drive','steer','scan','odom','joints'):
            if key not in state or time.monotonic()-state[key][0]>.25:raise RuntimeError(key+' stale/missing')
        rc=state['rc'][1];raw=state['raw'][1]
        channels={name:float(getattr(raw,name)) for name in
                  ('left_x','left_y','right_x','right_y','dial')}
        invalid=[name for name,value in channels.items() if not math.isfinite(value)]
        displaced=[name for name,value in channels.items() if abs(value)>.05]
        if not rc.connected or rc.emergency_stop or not rc.navigation_enabled or raw.online!=1 or invalid or displaced:
            report['rc_stop_evidence']={
                'time':time.monotonic(),'raw_online':int(raw.online),'channels':channels,
                'raw_receive_age_s':time.monotonic()-state['raw'][0],
                'unified_receive_age_s':time.monotonic()-state['rc'][0],
                'connected':bool(rc.connected),'emergency_stop':bool(rc.emergency_stop),
                'navigation_enabled':bool(rc.navigation_enabled),
                'invalid_channels':invalid,'displaced_channels':displaced,
                'recent_raw_anomalies':list(rc_anomalies)}
            if raw.online!=1:raise RuntimeError('Raw RC offline')
            if not rc.connected or rc.emergency_stop or not rc.navigation_enabled:raise RuntimeError('RC authorization revoked')
            if invalid:raise RuntimeError('RC invalid channel data: '+','.join(invalid))
            raise RuntimeError('RC channel outside neutral threshold: '+','.join(displaced))
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
        if len(ranges)<100 or min(ranges)<.4:
            nearest=min(((r,i) for i,r in enumerate(scan.ranges)
                         if math.isfinite(r) and scan.range_min<=r<=scan.range_max),default=None)
            report['scan_stop_evidence']={
                'time':time.monotonic(),'frame':scan.header.frame_id,
                'valid_beams':len(ranges),'nearest_m':nearest[0] if nearest else None,
                'nearest_bearing_rad':scan.angle_min+nearest[1]*scan.angle_increment if nearest else None,
                'receive_age_s':time.monotonic()-state['scan'][0],
                'angle_min':scan.angle_min,'angle_increment':scan.angle_increment,
                'ranges':[r if math.isfinite(r) else None for r in scan.ranges]}
            if len(ranges)<100:raise RuntimeError('Insufficient valid scan beams')
            if publisher:send()  # Stop immediately, before remaining guard checks.
            if not allow_obstacle_wait:raise RuntimeError('Obstacle within 0.4 m of base frame origin')
        if len(hardware.get_publishers_info_by_topic('/cmd_vel'))>(1 if publisher else 0):raise RuntimeError('Competing velocity publisher')
        p=position()
        if args.saved_goal:
            for key in ('local_costmap','global_costmap'):
                if key not in state or time.monotonic()-state[key][0]>(.75 if args.max_speed>.30 else 1.5):
                    raise RuntimeError(key+' stale/missing')
            for key in ('terrain_map','terrain_map_ext'):
                if key not in state or time.monotonic()-state[key][0]>.5:
                    raise RuntimeError(key+' stale/missing')
                stamp=state[key][1].header.stamp
                age=nav.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
                if not -.05<=age<.5:raise RuntimeError(key+' timestamp stale')
            if 'map' not in state or not path_is_free(state['map'][1],p,p):
                raise RuntimeError('Outside saved-map known-free area')
        if args.face_motion:
            # Hub explicitly delegates this topic to navigation while nav is enabled.
            # Use only its existing OFF command, never change RC or enable firing.
            allowed={'universal_controller_hub',hardware.get_name()}
            if any(p.node_name not in allowed for p in hardware.get_publishers_info_by_topic('/auto_aim_switch')):
                raise RuntimeError('Competing navigation autoaim permission publisher')
            if autoaim_gate_publisher and 'aim_switch' in state and state['aim_switch'][1].data!=0:
                raise RuntimeError('Autoaim permission re-enabled during heading control')
            if len(hardware.get_publishers_info_by_topic('/gimbal_scan_cmd'))>(1 if gimbal_publisher else 0):
                raise RuntimeError('Competing gimbal publisher')
            if 'gimbal_imu' not in state or time.monotonic()-state['gimbal_imu'][0]>.25:
                raise RuntimeError('Gimbal IMU stale/missing')
            imu=state['gimbal_imu'][1];stamp=imu.header.stamp
            age=hardware.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
            if not -.05<=age<.25 or abs(imu.angular_velocity.z)>.8:raise RuntimeError('Gimbal IMU age/rate gate')
            if args.align_only and origin and math.dist(p,origin)>.10:raise RuntimeError('Chassis moved during heading-only test')
        if not args.saved_goal and origin and math.dist(p,origin)>.35:raise RuntimeError('Trial displacement bound exceeded')
        return p
    def send(vx=0.,vy=0.):
        nonlocal last_sent_speed,last_sent_time
        msg=TwistStamped();msg.header.stamp=hardware.get_clock().now().to_msg();msg.header.frame_id='gimbal_yaw'
        # Existing Hub negates both navigation axes. Match its input contract
        # without changing RC arbitration; confirmed by two-axis real pulses.
        msg.twist.linear.x=-vx;msg.twist.linear.y=-vy;publisher.publish(msg)
        last_sent_speed=math.hypot(vx,vy);last_sent_time=time.monotonic()
    def stop(seconds=2.):
        end=time.monotonic()+seconds;next_send=0.
        while time.monotonic()<end:
            spin()
            if time.monotonic()>=next_send:
                send();next_send=time.monotonic()+.05
                sample={'time':time.monotonic(),'command':[0.,0.]}
                try:sample['pose']=position()
                except Exception:pass
                if 'drive' in state:
                    sample['wheel_speeds']=[getattr(state['drive'][1],f'motor{i}_speed') for i in range(1,5)]
                    sample['feedback_age_s']=time.monotonic()-state['drive'][0]
                report.setdefault('stop_samples',[]).append(sample)
    # Keep ROS contexts alive until zero commands and cancellation are delivered.
    def interrupted(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        save_report()
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:spin()
        if args.saved_goal:
            deadline=time.monotonic()+15
            while True:
                guard(allow_localization_wait=True)  # Still no actuator publisher.
                if state['localization_ready'][1].data:break
                if time.monotonic()>deadline:raise RuntimeError('Localization not stable during preflight')
                spin()
        origin=guard()
        frames=yaml.safe_load(buffer.all_frames_as_yaml())
        if frames.get('front_mid360',{}).get('parent')!='gimbal_yaw':
            raise RuntimeError('Acceptance requires confirmed gimbal-mounted lidar model')
        if not client.wait_for_server(timeout_sec=2):raise RuntimeError('NavigateToPose unavailable')
        tf=buffer.lookup_transform('map','base_footprint',Time());q=tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        publisher=hardware.create_publisher(TwistStamped,'/cmd_vel',10)
        if args.face_motion:
            autoaim_gate_publisher=hardware.create_publisher(Int32,'/auto_aim_switch',1)
            end=time.monotonic()+.3;tick=0.
            while time.monotonic()<end:
                spin();guard();send()
                if time.monotonic()>=tick:autoaim_gate_publisher.publish(Int32(data=0));tick=time.monotonic()+.05
            q=state['gimbal_imu'][1].orientation
            heading=HeadingTarget(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                                  -math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))))
            gimbal_publisher=hardware.create_publisher(GimbalCmd,'/gimbal_scan_cmd',10)
            report['face_motion']=True;report['alignment_only']=args.align_only
        # Automatic local-map targets; no hand-entered world coordinates.
        offsets=[('forward',.20,0),('home',0,0)] if args.course=='outback' else [('forward',.20,0),('corner',.20,.20),('left',0,.20),('home',0,0)]
        targets=[(name,origin[0]+dx*math.cos(yaw)-dy*math.sin(yaw),origin[1]+dx*math.sin(yaw)+dy*math.cos(yaw)) for name,dx,dy in offsets]
        if args.landmarks:
            saved=json.loads(args.landmarks.read_text())
            if args.saved_goal:
                validate_selected(saved,state['map'][1])
                report['scope']='Stationary saved-map localization and one continuously replanned goal; no gimbal scan'
                report['saved_map_sha256']=saved['map_sha256']
            if saved['frame']!='map':raise RuntimeError('Landmark frame must be map')
            targets=[(t['name'],float(t['x']),float(t['y'])) for t in saved['targets']]
            if args.saved_goal and not path_is_free(state['map'][1],targets[0][1:],targets[0][1:],radius=.4):
                raise RuntimeError('Selected goal lacks known free clearance')
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
            result=active.get_result_async();deadline=time.monotonic()+(240 if args.saved_goal else 30);next_send=0.;best=math.dist(position(),(gx,gy));progress=time.monotonic()
            previous=position();travelled=0.
            last_heading_update=time.monotonic();aligned_since=None;heading_deadline=time.monotonic()+20
            localization_pause_since=None
            obstacle_hold=ObstacleHold()
            while not result.done():
                spin();p=guard(allow_localization_wait=args.saved_goal,allow_obstacle_wait=args.saved_goal);distance=math.dist(p,(gx,gy))
                if args.stop_after_travel is not None and math.dist(p,origin)>=args.stop_after_travel:
                    report['controlled_stop_requested']={'time':time.monotonic(),'pose':p,'command_speed_mps':last_sent_speed}
                    raise RuntimeError('Planned controlled stopping measurement; destination not attempted')
                if distance<best-.01:best=distance;progress=time.monotonic()
                if args.saved_goal:
                    step=math.dist(p,previous);previous=p;travelled+=step
                    if step>.30 or travelled>20:raise RuntimeError('Pose jump / 20 m travel budget exceeded')
                elif distance> .30:raise RuntimeError('Motion diverged from target')
                if time.monotonic()>deadline or (not args.saved_goal and time.monotonic()-progress>8):raise RuntimeError('Navigation progress timeout')
                if args.saved_goal:
                    now=time.monotonic();scan=state['scan'][1]
                    nearest=min(r for r in scan.ranges if math.isfinite(r) and scan.range_min<=r<=scan.range_max)
                    was_holding=obstacle_hold.started is not None
                    holding=obstacle_hold.update(now,nearest,state['localization_ready'][1].data)
                    if holding:
                        if not was_holding:
                            report.setdefault('obstacle_pauses',[]).append({'start':now,'pose':p,
                                'trigger':copy.deepcopy(report.get('scan_stop_evidence'))})
                        if now>=next_send:
                            send();next_send=now+.05
                            if heading:
                                autoaim_gate_publisher.publish(Int32(data=0))
                                gimbal_publisher.publish(heading.message(hardware.get_clock().now().to_msg()))
                            report['trace'].append({'time':now,'target':name,'pose':p,'distance':distance,
                                'command':[0.,0.],'hub_command':[0.,0.],'obstacle_wait':True,'nearest_scan_m':nearest})
                        if now-last_checkpoint>1.:
                            save_report();last_checkpoint=now
                        last_heading_update=now
                        continue  # All guards ran; no nonzero command while held.
                    if was_holding:report['obstacle_pauses'][-1]['end']=now
                if args.saved_goal and not state['localization_ready'][1].data:
                    now=time.monotonic()
                    if localization_pause_since is None:
                        localization_pause_since=now
                        report.setdefault('localization_pauses',[]).append({'start':now,'pose':p})
                    if now-localization_pause_since>10.:
                        raise RuntimeError('Localization did not recover within 10 s at zero velocity')
                    if now>=next_send:
                        send();next_send=now+.05
                        if heading:
                            autoaim_gate_publisher.publish(Int32(data=0))
                            gimbal_publisher.publish(heading.message(hardware.get_clock().now().to_msg()))
                        report['trace'].append({'time':now,'target':name,'pose':p,'distance':distance,
                            'command':[0.,0.],'hub_command':[0.,0.],'localization_wait':True})
                    if now-last_checkpoint>1.:
                        save_report();last_checkpoint=now
                    last_heading_update=now
                    continue  # All hardware guards above still run on every iteration.
                if localization_pause_since is not None:
                    report['localization_pauses'][-1]['end']=time.monotonic()
                    localization_pause_since=None
                if time.monotonic()>=next_send:
                    vx=vy=0.
                    if 'command' in state and time.monotonic()-state['command'][0]<.2:
                        cmd=state['command'][1]
                        if abs(cmd.twist.angular.z)>.01:raise RuntimeError('Unexpected Nav2 rotation')
                        vx,vy=cmd.twist.linear.x,cmd.twist.linear.y
                        norm=math.hypot(vx,vy)
                        if not math.isfinite(norm):raise RuntimeError('Non-finite navigation command')
                        if norm>args.max_speed:vx*=args.max_speed/norm;vy*=args.max_speed/norm
                    heading_error=None
                    if heading:
                        autoaim_gate_publisher.publish(Int32(data=0))
                        now=time.monotonic();q=state['gimbal_imu'][1].orientation
                        imu_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                        heading_error=heading.update(imu_yaw,(vx,vy),now-last_heading_update);last_heading_update=now
                        gimbal_publisher.publish(heading.message(hardware.get_clock().now().to_msg()))
                        aligned=heading_error is not None and abs(heading_error)<.15
                        aligned_since=(aligned_since or now) if aligned else None
                        if args.align_only or heading_error is None or abs(heading_error)>.25:vx=vy=0.
                        if args.align_only and now>heading_deadline:raise RuntimeError('Heading alignment timeout')
                    vx,vy=limited_velocity(vx,vy,args.max_speed,last_sent_speed,
                        time.monotonic()-last_sent_time,distance)
                    if args.saved_goal:
                        tf=buffer.lookup_transform('map','gimbal_yaw',Time());q=tf.transform.rotation
                        a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                        # Nav2 handles obstacle/inflation costs and replanning.
                        # Do not demand a fixed straight corridor for curved paths.
                        projected=(p[0]+.5*(vx*math.cos(a)-vy*math.sin(a)),p[1]+.5*(vx*math.sin(a)+vy*math.cos(a)))
                        if not path_is_free(state['map'][1],p,projected):raise RuntimeError('Command exits known free map')
                    send(vx,vy);next_send=time.monotonic()+.05
                    report['trace'].append({'time':time.monotonic(),'target':name,'pose':p,'distance':distance,'command':[vx,vy],'hub_command':[-vx,-vy]})
                    if heading:
                        report['trace'][-1].update(heading_error_rad=heading_error,gimbal_imu_yaw=imu_yaw,gimbal_target_yaw=heading.yaw)
                    if time.monotonic()-last_checkpoint>1.:
                        q=buffer.lookup_transform('map','base_footprint',Time()).transform.rotation
                        report['trace'][-1]['chassis_yaw_rad']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                        report['trace'][-1]['wheel_speeds']=[getattr(state['drive'][1],f'motor{i}_speed') for i in range(1,5)]
                        scan=state['scan'][1]
                        report['trace'][-1]['nearest_scan_m']=min(r for r in scan.ranges if math.isfinite(r) and scan.range_min<=r<=scan.range_max)
                        if 'plan' in state:
                            plan=state['plan'][1]
                            report.setdefault('plans',[]).append({'time':time.monotonic(),'frame':plan.header.frame_id,
                                'points':[[v.pose.position.x,v.pose.position.y] for v in plan.poses]})
                        if args.saved_goal and 'global_costmap' in state:
                            grid=state['global_costmap'][1]
                            if grid.header.frame_id=='map':
                                # Evidence only: retain nearby lethal cells so a
                                # replanned path can be checked against real obstacles.
                                ox=grid.info.origin.position.x;oy=grid.info.origin.position.y
                                resolution=grid.info.resolution;width=grid.info.width
                                cells=[]
                                for index,cost in enumerate(grid.data):
                                    if cost!=100:continue
                                    x=ox+(index%width+.5)*resolution
                                    y=oy+(index//width+.5)*resolution
                                    if math.hypot(x-p[0],y-p[1])<3.:cells.append([x,y])
                                report.setdefault('costmap_obstacles',[]).append({
                                    'time':time.monotonic(),'frame':'map','resolution':resolution,
                                    'receive_age_s':time.monotonic()-state['global_costmap'][0],
                                    'lethal_cells_within_3m':cells})
                        save_report();last_checkpoint=time.monotonic()
                    if args.align_only and aligned_since and time.monotonic()-aligned_since>.5:
                        report['heading_alignment_passed']=True
                        break
            if args.align_only:
                if not report.get('heading_alignment_passed'):raise RuntimeError('Heading test ended without alignment')
                break
            status=result.result().status;active=None;stop()
            error=math.dist(position(),(gx,gy))
            report['waypoints'].append({'name':name,'target':[gx,gy],'status':status,'position_error_m':error})
            if status!=4 or error>.06:raise RuntimeError('Goal did not meet measured tolerance')
        report['local_navigation_passed']=not args.align_only
    except (Exception,KeyboardInterrupt) as exc:
        report['abort_reason']=str(exc) or 'Operator interrupted'
    finally:
        if active:
            try:active.cancel_goal_async()
            except Exception:pass
        if publisher:stop()
        report['final_wheel_speeds']=[getattr(state['drive'][1],f'motor{i}_speed') for i in range(1,5)] if 'drive' in state else None
        report['final_wheel_feedback_age_s']=time.monotonic()-state['drive'][0] if 'drive' in state else None
        report['recent_rc_anomalies']=list(rc_anomalies)
        report['status']='aborted' if 'abort_reason' in report else 'completed'
        save_report()
        print(json.dumps({k:v for k,v in report.items() if k not in ('trace','plans','costmap_obstacles','stop_samples','scan_stop_evidence')}))
        exlive.shutdown();exnav.shutdown()
        for worker in workers:worker.join()
        hardware.destroy_node();nav.destroy_node();live.try_shutdown();test.try_shutdown()
    return 0 if report.get('local_navigation_passed') or report.get('heading_alignment_passed') else 1


if __name__=='__main__':raise SystemExit(main())
