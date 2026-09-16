#!/usr/bin/env python3
"""Compute four diagnostic paths in isolated domain 88. Never follow paths."""
import json
import math
import os
from pathlib import Path
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='88':raise RuntimeError('Isolated domain 88 required')
    rclpy.init();node=rclpy.create_node('acceptance_plan_probe',enable_rosout=False,start_parameter_services=False)
    buffer=Buffer();listener=TransformListener(buffer,node)
    client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    report={'motion_authorized':False,'physical_acceptance_passed':False,'paths':[],
            'scope':'Local diagnostic map and model envelope; not saved-field localization or physical tracking.'}
    def wait(future,seconds=3):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not future.done():raise RuntimeError('Planner timeout')
        return future.result()
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        tf=buffer.lookup_transform('map','base_footprint',Time())
        q=tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        report['base_pose_map'] = [tf.transform.translation.x,tf.transform.translation.y,yaw]
        if not client.wait_for_server(timeout_sec=2):raise RuntimeError('Planner unavailable')
        for name,dx,dy in [('forward',.25,0),('backward',-.25,0),('left',0,.25),('right',0,-.25)]:
            goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=False
            goal.goal.header.frame_id='map';goal.goal.header.stamp=node.get_clock().now().to_msg()
            goal.goal.pose.position.x=tf.transform.translation.x+dx*math.cos(yaw)-dy*math.sin(yaw)
            goal.goal.pose.position.y=tf.transform.translation.y+dx*math.sin(yaw)+dy*math.cos(yaw)
            goal.goal.pose.orientation.w=1.
            handle=wait(client.send_goal_async(goal))
            row={'direction':name,'target':[goal.goal.pose.position.x,goal.goal.pose.position.y], 'accepted':handle.accepted}
            if handle.accepted:
                try:result=wait(handle.get_result_async())
                except RuntimeError:
                    wait(handle.cancel_goal_async());raise
                row.update(status=result.status,frame=result.result.path.header.frame_id,
                           points=[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses])
            report['paths'].append(row)
    except (RuntimeError,TransformException) as exc:report['error']=str(exc)
    finally:
        Path('/tmp/sentry-static-acceptance/planning.json').write_text(json.dumps(report,indent=2)+'\n')
        print('Diagnostic paths:',[(r['direction'],r.get('status'),len(r.get('points',[]))) for r in report['paths']],report.get('error',''))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
