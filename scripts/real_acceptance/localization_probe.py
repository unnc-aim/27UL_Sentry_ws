#!/usr/bin/env python3
"""Global AMCL initialization and read-only evidence; never commands actuators."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time
import threading
from rclpy.executors import SingleThreadedExecutor
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import ParticleCloud
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from rclpy.serialization import deserialize_message
from tf2_ros import Buffer, TransformListener


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))



def aligned_pair(poses, scans, tolerance=.025):
    """Newest available measured pair; callers must separately enforce freshness."""
    scans=list(scans)
    if not scans:return None
    for pose in reversed(list(poses)):
        def delta(scan):
            return (scan.header.stamp.sec-pose.header.stamp.sec)+(scan.header.stamp.nanosec-pose.header.stamp.nanosec)*1e-9
        scan=min(scans,key=lambda item:abs(delta(item)))
        if abs(delta(scan))<=tolerance:return pose,scan
    return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=30)
    parser.add_argument('--globalize',action='store_true')
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'results/global_localization.json')
    args=parser.parse_args();rclpy.init();n=rclpy.create_node('acceptance_localization_probe')
    state={};trace=[]
    latest_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    scans=deque(maxlen=5000)
    poses=deque(maxlen=100)
    def receive(k,msg):
        state[k]=(time.monotonic(),msg)
        if k=='pose':poses.append(msg)
        if k=='scan':
            scans.append(msg)
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            while scans and stamp-(scans[0].header.stamp.sec+scans[0].header.stamp.nanosec*1e-9)>2.:
                scans.popleft()
    for topic,k,typ,qos in [('/amcl_pose','pose',PoseWithCovarianceStamped,qos_profile_sensor_data),('/obstacle_scan','scan',LaserScan,qos_profile_sensor_data),('/map','map',OccupancyGrid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
        n.create_subscription(typ,topic,lambda m,k=k:receive(k,m),qos)
    last_particle_decode=[0.]
    def receive_particles(raw):
        if time.monotonic()-last_particle_decode[0]>=.75:
            last_particle_decode[0]=time.monotonic()
            receive('particles',deserialize_message(raw,ParticleCloud))
    n.create_subscription(ParticleCloud,'/particle_cloud',receive_particles,latest_qos,raw=True)
    ready_publisher=n.create_publisher(Bool,'/acceptance/localization_ready',1)
    globalize=n.create_client(Empty,'/reinitialize_global_localization');nomotion=n.create_client(Empty,'/request_nomotion_update')
    executor=SingleThreadedExecutor();executor.add_node(n)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    report={'motion_authorized':False,'globalization_requested':False,'trace':trace}
    def spin_until(future,seconds):
        until=time.monotonic()+seconds
        while not future.done() and time.monotonic()<until:time.sleep(.02)
        if not future.done():raise RuntimeError('Localization service timeout')
        future.result()
    try:
        if args.globalize:
            if not globalize.wait_for_service(timeout_sec=10):raise RuntimeError('Global localization unavailable')
            spin_until(globalize.call_async(Empty.Request()),5);report['globalization_requested']=True
        end=time.monotonic()+args.seconds;tick=0;mask=None;good_since=None
        while time.monotonic()<end:
            time.sleep(.02)
            if time.monotonic()<tick:continue
            tick=time.monotonic()+.5
            if nomotion.service_is_ready():nomotion.call_async(Empty.Request())
            if not all(k in state for k in ('pose','map','scan','particles')):continue
            m=state['map'][1];pose=state['pose'][1];cloud=state['particles'][1]
            pair=aligned_pair(poses,scans)
            if pair is None:
                ready_publisher.publish(Bool(data=False));good_since=None;continue
            pose,scan=pair
            if mask is None:
                occupied=np.asarray(m.data).reshape(m.info.height,m.info.width)>=65
                radius=math.ceil(.15/m.info.resolution);pad=np.pad(occupied,radius);mask=np.zeros_like(occupied)
                for dy in range(-radius,radius+1):
                    for dx in range(-radius,radius+1):
                        if math.hypot(dx,dy)*m.info.resolution<=.15:
                            mask |= pad[radius+dy:radius+dy+m.info.height,radius+dx:radius+dx+m.info.width]
            row={'time':time.time(),'candidate_stable':False}
            try:
                # AMCL pose already expresses base_footprint in map. Require matching
                # scan frame and bound measurement-time difference explicitly.
                if scan.header.frame_id!='base_footprint':raise ValueError('Unexpected scan frame')
                offset=(pose.header.stamp.sec-scan.header.stamp.sec)+(pose.header.stamp.nanosec-scan.header.stamp.nanosec)*1e-9
                if abs(offset)>.075:raise ValueError(f'No matching scan: offset={offset:.3f}s, buffer={len(scans)}')
                p=pose.pose.pose.position;a=yaw(pose.pose.pose.orientation)
                r=np.asarray(scan.ranges)[::4];angles=scan.angle_min+np.arange(len(scan.ranges))[::4]*scan.angle_increment+a
                valid=np.isfinite(r)&(r>scan.range_min)&(r<scan.range_max)
                x=p.x+r[valid]*np.cos(angles[valid]);y=p.y+r[valid]*np.sin(angles[valid])
                if abs(yaw(m.info.origin.orientation))>1e-6:raise ValueError('Rotated map origin unsupported')
                ix=np.floor((x-m.info.origin.position.x)/m.info.resolution).astype(int);iy=np.floor((y-m.info.origin.position.y)/m.info.resolution).astype(int)
                inside=(ix>=0)&(iy>=0)&(ix<m.info.width)&(iy<m.info.height);hits=np.zeros(len(ix),dtype=bool);hits[inside]=mask[iy[inside],ix[inside]]
                match=float(np.mean(hits)) if len(hits)>=30 else 0.
                pp=pose.pose.pose.position;heading=yaw(pose.pose.pose.orientation);weights=[max(0.,v.weight) for v in cloud.particles];total=sum(weights)
                mass=sum(w for w,v in zip(weights,cloud.particles) if math.hypot(v.pose.position.x-pp.x,v.pose.position.y-pp.y)<.35 and abs(math.atan2(math.sin(yaw(v.pose.orientation)-heading),math.cos(yaw(v.pose.orientation)-heading)))<.3)/total if total else 0.
                cov=[pose.pose.covariance[i] for i in (0,7,35)]
                now=n.get_clock().now().nanoseconds*1e-9
                age=now-pose.header.stamp.sec-pose.header.stamp.nanosec*1e-9
                scan_age=now-scan.header.stamp.sec-scan.header.stamp.nanosec*1e-9
                good=(-.05<age<.75 and -.05<scan_age<.75 and time.monotonic()-state['scan'][0]<.3 and time.monotonic()-state['particles'][0]<2 and match>.85 and mass>.95 and cov[0]<.04 and cov[1]<.04 and cov[2]<.025)
                good_since=(good_since or time.monotonic()) if good else None
                row.update(particle_receive_age_s=time.monotonic()-state['particles'][0],scan_receive_age_s=time.monotonic()-state['scan'][0],pose_age_s=age,scan_age_s=scan_age,pose_scan_offset_s=offset,pose=[pp.x,pp.y,heading],covariance=cov,scan_match=match,dominant_particle_mass=mass,particle_count=len(weights),candidate_stable=bool(good_since and time.monotonic()-good_since>3))
            except Exception as exc:row['error']=str(exc);good_since=None
            trace.append(row)
            ready_publisher.publish(Bool(data=row['candidate_stable']))
            report['latest']=row
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(report,indent=2)+'\n')
        report['latest']=trace[-1] if trace else {'candidate_stable':False,'reason':'Insufficient observations'}
    except (Exception,KeyboardInterrupt) as exc:report['error']=str(exc) or 'Interrupted'
    finally:
        try:ready_publisher.publish(Bool(data=False))
        except Exception:pass
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='trace'}));executor.shutdown();worker.join();n.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
