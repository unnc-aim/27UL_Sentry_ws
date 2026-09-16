#!/usr/bin/env python3
"""Plan and validate short map segments to a selected goal; no actuator commands."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from startup_trial import path_is_free


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('selected',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();selected=json.loads(args.selected.read_text());rclpy.init();n=rclpy.create_node('selected_goal_planner');state={}
    n.create_subscription(OccupancyGrid,'/map',lambda m:state.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(Bool,'/acceptance/localization_ready',lambda m:state.update(ready=(time.monotonic(),m.data)),1)
    client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
    def wait(future,seconds=10):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
        if not future.done():raise RuntimeError('Planner timeout')
        return future.result()
    try:
        end=time.monotonic()+20
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.01)
            if 'map'in state and state.get('ready',(0,False))[1] and time.monotonic()-state['ready'][0]<1:break
        else:raise RuntimeError('Localization not ready; no route authorized')
        if selected['frame']!='map' or len(selected['targets'])!=1:raise ValueError('Select one map goal')
        target=selected['targets'][0];g=ComputePathToPose.Goal();g.planner_id='GridBased';g.goal.header.frame_id='map';g.goal.header.stamp=n.get_clock().now().to_msg();g.goal.pose.position.x=target['x'];g.goal.pose.position.y=target['y'];g.goal.pose.orientation.w=1.
        if not client.wait_for_server(timeout_sec=3):raise RuntimeError('Planner unavailable')
        h=wait(client.send_goal_async(g))
        if not h.accepted:raise RuntimeError('Planning rejected')
        result=wait(h.get_result_async())
        if result.status!=4 or result.result.path.header.frame_id!='map':raise RuntimeError('Planning failed')
        points=[(p.pose.position.x,p.pose.position.y) for p in result.result.path.poses]
        if len(points)<2:raise RuntimeError('Empty route')
        if sum(math.dist(a,b) for a,b in zip(points,points[1:]))>5:raise RuntimeError('Route exceeds 5 m acceptance scope')
        targets=[];last=points[0]
        for point in points[1:]:
            if math.dist(last,point)>=.15:
                if math.dist(last,point)>.25 or not path_is_free(state['map'],last,point):raise RuntimeError('Segment lacks known free clearance')
                targets.append({'name':f'segment_{len(targets)+1}','x':point[0],'y':point[1]});last=point
        goal=(target['x'],target['y'])
        if not path_is_free(state['map'],last,goal):raise RuntimeError('Final goal corridor invalid')
        targets.append({'name':'selected_goal','x':goal[0],'y':goal[1]})
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps({'frame':'map','targets':targets,'source':str(args.selected),'path':points,'motion_authorized':False},indent=2)+'\n')
        print('Validated segments:',len(targets),'route:',args.output)
    finally:n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
