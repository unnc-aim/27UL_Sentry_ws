#!/usr/bin/env python3
"""Record RViz map goals only. No navigation actions or actuator publishers."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from startup_trial import path_is_free


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    args=parser.parse_args();folder=args.map.resolve().parent
    rclpy.init();node=rclpy.create_node('acceptance_goal_selector')
    grid=[None]
    node.create_subscription(OccupancyGrid,'/map',lambda msg:grid.__setitem__(0,msg),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    marker=node.create_publisher(Marker,'/acceptance/selected_goal_marker',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def selected(msg):
        x,y=msg.pose.position.x,msg.pose.position.y
        valid=(msg.header.frame_id=='map' and math.isfinite(x) and math.isfinite(y) and grid[0] is not None
               and path_is_free(grid[0],(x,y),(x,y),radius=.4))
        m=Marker();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg()
        m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose.position.x=x;m.pose.position.y=y
        m.pose.position.z=.1;m.pose.orientation.w=1.;m.scale.z=.18;m.color.a=1.
        m.color.g=1. if valid else 0.;m.color.r=0. if valid else 1.
        m.text=f'{"Selected" if valid else "Rejected"}: ({x:.2f}, {y:.2f})';marker.publish(m)
        if not valid:
            (folder/'selected_goal.json').unlink(missing_ok=True)
            node.get_logger().warn('Goal rejected: require map-frame known free disc of radius 0.4 m')
            return
        import yaml
        meta=yaml.safe_load(args.map.read_text());image=folder/meta['image']
        result={'frame':'map','map_yaml':str(args.map.resolve()),'map_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
                'targets':[{'name':'selected_goal','x':x,'y':y}], 'motion_authorized':False}
        (folder/'selected_goal.json').write_text(json.dumps(result,indent=2)+'\n')
        node.get_logger().info(f'Saved goal ({x:.3f}, {y:.3f}); no navigation sent')
    node.create_subscription(PoseStamped,'/acceptance/selected_goal',selected,10)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
