#!/usr/bin/env python3
"""Explicit single low-speed interface pulse. Never changes RC/estop or control modes."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from pb_rm_interfaces.msg import GimbalCmd
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from universal_controller.msg import UnifiedInput
from chassis_controllers.msg import ChassisControl
from custom_msgs.msg import ReadLkMotorMulti, ReadDJIRC


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Required after on-site clearance and RC confirmation')
    parser.add_argument('--duration', type=float, default=.5)
    parser.add_argument('--speed', type=float, default=.1)
    parser.add_argument('--axis', choices=['x','y','yaw','gimbal_yaw'], default='x')
    parser.add_argument('--sign', type=int, choices=[-1,1], default=1)
    parser.add_argument('--turns', type=int, choices=[0,1,2], default=0)
    parser.add_argument('--continuous', action='store_true', help='Gimbal cable clearance must be confirmed')
    args = parser.parse_args()
    if args.continuous and args.axis!='gimbal_yaw':parser.error('--continuous requires gimbal_yaw')
    if args.turns and args.axis not in ('yaw','gimbal_yaw'):parser.error('--turns requires yaw or gimbal_yaw')
    if not .05 <= args.speed <= (.4 if args.axis in ('yaw','gimbal_yaw') else .15):
        parser.error('Invalid speed: maximum .15 m/s translation or .4 rad/s yaw')
    if not .1 <= args.duration <= 3.0:
        parser.error('Duration must be between 0.1 and 3.0 seconds')
    if not args.execute:
        parser.error('No motion without --execute and on-site confirmation')
    live, test = Context(), Context()
    os.environ['ROS_LOCALHOST_ONLY'] = '0'; live.init(domain_id=0)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'; test.init(domain_id=88)
    reader = rclpy.create_node('acceptance_motion_probe', context=live, enable_rosout=False, start_parameter_services=False)
    observer = rclpy.create_node('acceptance_motion_observer', context=test, enable_rosout=False, start_parameter_services=False)
    ex_live, ex_test = SingleThreadedExecutor(context=live), SingleThreadedExecutor(context=test)
    ex_live.add_node(reader); ex_test.add_node(observer)
    buffer = Buffer(node=observer)
    listener = TransformListener(buffer, observer)
    state = {}
    def receive(name, message):
        state[name] = (time.monotonic(), message)
    for node, name, topic, cls in [
        (reader, 'raw_rc', '/ecat/sn4653115/app1/read', ReadDJIRC),
        (reader, 'drive', '/ecat/sn2228292/app1/read', ReadLkMotorMulti),
        (reader, 'steer', '/ecat/sn2228292/app2/read', ReadLkMotorMulti),
        (reader, 'rc', '/universal_controller/input/ndj', UnifiedInput),
        (reader, 'chassis', '/chassis_command', ChassisControl),
        (observer, 'odom', '/odometry', Odometry),
        (observer, 'scan', '/obstacle_scan', LaserScan),
    ]:
        node.create_subscription(cls, topic, lambda msg,k=name:receive(k,msg), qos_profile_sensor_data)
    def spin():
        ex_live.spin_once(timeout_sec=.002); ex_test.spin_once(timeout_sec=.002)
    def ready():
        now = time.monotonic()
        for k in ('raw_rc','rc','chassis','odom','scan','drive','steer'):
            if k not in state or now-state[k][0] > .25:
                return k+' missing/stale'
        raw = state['raw_rc'][1]
        if not raw.online or any(abs(v) > .05 for v in (raw.left_x,raw.left_y,raw.right_x,raw.right_y,raw.dial)):
            return 'Raw RC input not neutral'
        for key in ('drive','steer'):
            if not all(getattr(state[key][1], f'motor{i}_online') for i in range(1,5)):
                return key+' motor offline'
        rc = state['rc'][1]
        if not rc.connected or rc.emergency_stop or not rc.navigation_enabled:
            return 'RC not authorizing navigation'
        if any(abs(v) > .001 for v in (rc.vx,rc.vy,rc.wz)) or rc.fire_trigger or rc.friction_on:
            return 'RC sticks/spin/fire not neutral'
        if state['chassis'][1].emergency_stop:
            return 'Chassis emergency stop active'
        # In navigation mode the interpreter zeros RC wz even with spin_mode set.
        # Gate the actual outgoing rotation as well as RC wz; never override it.
        if abs(state['chassis'][1].spin_speed) > (args.speed+.01 if args.axis=='yaw' else .001):
            return 'Actual chassis rotation command active'
        scan = state['scan'][1]
        if scan.header.frame_id != 'base_footprint':
            return 'Unexpected scan frame'
        ranges = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        # Radius .20 m confirmed by operator; reserve .20 m for short-pulse
        # latency/braking/measurement uncertainty. This is not a stopping-distance certification.
        if len(ranges) < 100 or min(ranges) < .4:
            return f'Insufficient scan clearance: finite_returns={len(ranges)}, closest_m={min(ranges) if ranges else None}'
        for k in ('odom','scan'):
            stamp = state[k][1].header.stamp
            age = observer.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
            if not -.05 <= age < .3:
                return k+' measurement timestamp stale'
        if len(reader.get_publishers_info_by_topic('/cmd_vel')) > (1 if publisher else 0):
            return 'Competing velocity publisher'
        if args.axis=='gimbal_yaw' and len(reader.get_publishers_info_by_topic('/gimbal_scan_cmd'))>(1 if scan_publisher else 0):
            return 'Competing gimbal scan publisher'
        try:
            tf = buffer.lookup_transform('odom', 'base_footprint', Time())
            stamp=tf.header.stamp
            age=observer.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
            if not -.05 <= age < .3:return 'Base TF stale'
        except TransformException:return 'Base TF missing'
        return None
    publisher = None
    scan_publisher = None
    def scan_command(speed):
        msg=GimbalCmd();msg.header.stamp=reader.get_clock().now().to_msg()
        msg.yaw_type=GimbalCmd.VELOCITY;msg.pitch_type=GimbalCmd.VELOCITY
        msg.velocity.yaw=speed;msg.velocity.pitch=0.
        limit=6*math.pi if args.continuous else math.pi
        msg.velocity.yaw_min_range=-limit;msg.velocity.yaw_max_range=limit
        msg.velocity.pitch_min_range=-.3;msg.velocity.pitch_max_range=.3
        scan_publisher.publish(msg)
    report = {'physical_acceptance_passed': False, 'pulse_sent': False, 'speed_mps': args.speed if args.axis in ('x','y') else 0., 'yaw_rate_radps': args.speed if args.axis=='yaw' else 0., 'duration_s': args.duration, 'requested_turns':args.turns, 'continuous':args.continuous, 'gimbal_scan_rate_radps':args.speed if args.axis=='gimbal_yaw' else 0., 'trace': [], 'axis': args.axis, 'sign': args.sign, 'radius_m': .2, 'stop_distance_from_origin_m': .1}
    output = Path(os.environ.get('SENTRY_ACCEPTANCE_OUTPUT', str(Path(__file__).parent/'results/current')))/'motion_probe.json'
    start = None
    epoch = time.monotonic()
    def record(stage):
        p = state['odom'][1].pose.pose
        c = state['chassis'][1]
        scan = state['scan'][1]
        valid = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        try:
            tf = buffer.lookup_transform('odom','base_footprint',Time())
            q=tf.transform.rotation
            body_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        except TransformException:body_yaw=None
        row = {'body_yaw': body_yaw, 'scan_finite_returns': len(valid), 'scan_closest_m': min(valid) if valid else None, 't': time.monotonic()-epoch, 'stage': stage,
               'position': [p.position.x,p.position.y],
               'orientation': [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],
               'chassis': [c.x_speed,c.y_speed,c.spin_speed]}
        for key in ('drive','steer'):
            row[key] = [[getattr(state[key][1], f'motor{i}_{field}') for field in ('encoder','speed','current')] for i in range(1,5)]
        try:
            qg=buffer.lookup_transform('odom','gimbal_yaw',Time()).transform.rotation
            row['gimbal_yaw']=math.atan2(2*(qg.w*qg.z+qg.x*qg.y),1-2*(qg.y*qg.y+qg.z*qg.z))
        except TransformException:row['gimbal_yaw']=None
        report['trace'].append(row)
    try:
        deadline=time.monotonic()+5
        next_baseline=deadline-1.0
        while time.monotonic()<deadline:
            spin()
            if time.monotonic()>=next_baseline and all(k in state for k in ('odom','chassis','drive','steer','scan')):
                record('baseline');next_baseline=time.monotonic()+.05
        error=ready()
        if error:
            raise RuntimeError(error)
        start=state['odom'][1].pose.pose.position
        publisher=reader.create_publisher(TwistStamped,'/cmd_vel',10)
        if args.axis=='gimbal_yaw':
            if reader.get_publishers_info_by_topic('/gimbal_scan_cmd'):
                raise RuntimeError('Competing gimbal scan publisher')
            scan_publisher=reader.create_publisher(GimbalCmd,'/gimbal_scan_cmd',10)
        record('before')
        deadline=time.monotonic()+(120 if args.turns else args.duration)
        rotation=0.
        previous_yaw=report['trace'][-1]['gimbal_yaw' if args.axis=='gimbal_yaw' else 'body_yaw']
        coverage=set()
        scan_progress=time.monotonic()
        scan_distance=0.
        next_send=0.
        while time.monotonic()<deadline:
            spin()
            error=ready()
            if error: raise RuntimeError(error)
            if args.turns:
                tf=buffer.lookup_transform('odom','gimbal_yaw' if args.axis=='gimbal_yaw' else 'base_footprint',Time());q=tf.transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                change=math.atan2(math.sin(yaw-previous_yaw),math.cos(yaw-previous_yaw))
                if abs(change)>.2:raise RuntimeError('Yaw discontinuity')
                if args.axis=='gimbal_yaw':
                    if abs(change)>.005:
                        rotation+=change if args.continuous else abs(change);previous_yaw=yaw
                    coverage.add(int(((yaw+math.pi)%(2*math.pi))/(2*math.pi)*24))
                    if abs(rotation)>scan_distance+.05:
                        scan_distance=abs(rotation);scan_progress=time.monotonic()
                    if time.monotonic()-scan_progress>8:raise RuntimeError('Gimbal scan made no measured progress')
                else:
                    rotation+=change;previous_yaw=yaw
                if abs(rotation)>=args.turns*2*math.pi and (args.axis!='gimbal_yaw' or len(coverage)>=23):
                    report['startup_turns_completed']=args.turns
                    report['scan_travel_rad']=abs(rotation)
                    report['scan_coverage_bins']=len(coverage)
                    break
            current=state['odom'][1].pose.pose.position
            if math.hypot(current.x-start.x,current.y-start.y)>.1:
                raise RuntimeError('Displacement limit exceeded')
            if time.monotonic()>=next_send:
                msg=TwistStamped();msg.header.stamp=reader.get_clock().now().to_msg()
                msg.header.frame_id='gimbal_yaw'
                if args.axis=='yaw':msg.twist.angular.z=args.sign*args.speed
                elif args.axis in ('x','y'):setattr(msg.twist.linear, args.axis, args.sign*args.speed)
                if scan_publisher:scan_command(args.sign*args.speed)
                record('pulse')
                publisher.publish(msg);report['pulse_sent']=True
                next_send=time.monotonic()+.05
        if args.turns and not report.get('startup_turns_completed'):
            raise RuntimeError('Startup rotation timeout')
    except (RuntimeError, KeyboardInterrupt) as exc:
        report['abort_reason']=str(exc) or 'Interrupted'
    finally:
        if publisher:
            # Zero only through the existing navigation input. RC/estop stays authoritative.
            deadline=time.monotonic()+2.0
            while time.monotonic()<deadline:
                msg=TwistStamped();msg.header.stamp=reader.get_clock().now().to_msg()
                msg.header.frame_id='gimbal_yaw';publisher.publish(msg)
                if scan_publisher:scan_command(0.)
                if all(k in state for k in ('odom','chassis','drive','steer')):record('stop')
                until=time.monotonic()+.05
                while time.monotonic()<until:spin()
        if start and 'odom' in state:
            end=state['odom'][1].pose.pose.position
            report['observed_odom_displacement_m']=[end.x-start.x,end.y-start.y]
        gimbal_angles=[x['gimbal_yaw'] for x in report['trace'] if x.get('gimbal_yaw') is not None]
        report['measured_gimbal_yaw_change_rad']=sum(math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(gimbal_angles,gimbal_angles[1:]))
        angles=[x['body_yaw'] for x in report['trace'] if x['body_yaw'] is not None]
        report['measured_yaw_change_rad']=sum(math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(angles,angles[1:]))
        before=[x['position'] for x in report['trace'] if x['stage']=='baseline']
        after=[x['position'] for x in report['trace'] if x['stage']=='stop'][-10:]
        if before and after:
            report['averaged_displacement_m']=[sum(p[i] for p in after)/len(after)-sum(p[i] for p in before)/len(before) for i in range(2)]
        report['final_wheel_speeds']=[getattr(state['drive'][1],f'motor{i}_speed') for i in range(1,5)] if 'drive' in state else None
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='trace'}))
        ex_live.shutdown();ex_test.shutdown();reader.destroy_node();observer.destroy_node()
        live.try_shutdown();test.try_shutdown()
    return 1 if 'abort_reason' in report else 0


if __name__=='__main__':
    raise SystemExit(main())
