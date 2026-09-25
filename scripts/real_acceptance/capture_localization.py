#!/usr/bin/env python3
"""Capture stationary scan/TF evidence only; no actuator or initial-pose output."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args=parser.parse_args();rclpy.init();n=rclpy.create_node('stationary_capture')
    buffer=Buffer(node=n);listener=TransformListener(buffer,n);state={};samples=[]
    for key,topic,typ in [('scan','/obstacle_scan',LaserScan),('pose','/amcl_pose',PoseWithCovarianceStamped),('joints','/joint_states',JointState)]:
        n.create_subscription(typ,topic,lambda m,k=key:state.update({k:m}),qos_profile_sensor_data)
    end=time.monotonic()+10;tick=0
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.01)
        if time.monotonic()<tick or 'scan' not in state:continue
        tick=time.monotonic()+.5
        m=state['scan'];stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if n.get_clock().now().nanoseconds*1e-9-stamp>.3:continue
        try:
            tf=buffer.lookup_transform('odom','base_footprint',Time.from_msg(m.header.stamp))
        except Exception:continue
        p=tf.transform.translation;q=tf.transform.rotation
        points=[[r*math.cos(m.angle_min+i*m.angle_increment),r*math.sin(m.angle_min+i*m.angle_increment)]
                for i,r in enumerate(m.ranges) if math.isfinite(r) and m.range_min<r<m.range_max]
        samples.append({'time':stamp,'frame':m.header.frame_id,'points':points,
            'odom_pose':[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],
            'gimbal_joint':list(state['joints'].position) if 'joints' in state else None})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({'samples':samples,'motion_commanded':False},indent=2)+'\n')
    print('Captured stationary samples:',len(samples),args.output)
    n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
