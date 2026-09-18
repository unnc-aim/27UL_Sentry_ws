#!/usr/bin/env python3
"""Fresh local mapping -> two measured turns -> saved-map goal, through original Hub."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def snapshot():
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node('startup_map_observer')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    maps = []
    node.create_subscription(OccupancyGrid, '/map', lambda msg: maps.append(msg),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic()+20
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            try:
                tf=buffer.lookup_transform('map','base_footprint',Time())
                age=node.get_clock().now().nanoseconds*1e-9-tf.header.stamp.sec-tf.header.stamp.nanosec*1e-9
                if maps and -.05<=age<.3:
                    p=tf.transform.translation;q=tf.transform.rotation
                    return maps[-1], (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            except Exception:
                pass
        raise RuntimeError('Fresh map/pose unavailable')
    finally:
        node.destroy_node();rclpy.shutdown()


def path_is_free(grid, start, goal, radius=.30):
    """Conservative swept disc: unknown or out-of-map cells reject the goal."""
    info=grid.info
    if grid.header.frame_id!='map' or info.resolution<=0: return False
    q=info.origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def local(p):
        x=p[0]-info.origin.position.x;y=p[1]-info.origin.position.y
        return x*math.cos(yaw)+y*math.sin(yaw),-x*math.sin(yaw)+y*math.cos(yaw)
    a=local(start);b=local(goal);res=info.resolution
    # Cell centers padded by half the diagonal, so intersected occupied cells count.
    pad=radius+res/math.sqrt(2)
    steps=max(1,math.ceil(math.dist(a,b)/(res/2)))
    for step in range(steps+1):
        x=a[0]+(b[0]-a[0])*step/steps;y=a[1]+(b[1]-a[1])*step/steps
        for iy in range(math.floor((y-pad)/res),math.floor((y+pad)/res)+1):
            for ix in range(math.floor((x-pad)/res),math.floor((x+pad)/res)+1):
                if math.hypot((ix+.5)*res-x,(iy+.5)*res-y)>pad:continue
                if not (0<=ix<info.width and 0<=iy<info.height):return False
                value=grid.data[iy*info.width+ix]
                if value<0 or value>=25:return False
    return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--lidar-calibration',type=Path)
    args=parser.parse_args()
    if not args.execute:parser.error('--execute required with on-site takeover ready')
    folder=Path(__file__).resolve().parent
    # Never stop or compete with a pre-existing sensor owner.
    running=subprocess.run(['pgrep','-f','[/]livox_ros_driver2_node|[/]pointlio_mapping'],capture_output=True,text=True)
    if running.returncode==0:raise SystemExit('Existing lidar/Point-LIO process; stop the owned diagnostic launch first')
    out=folder/'results'/time.strftime('startup_%Y%m%d_%H%M%S');out.mkdir()
    os.environ.update(ROS_DOMAIN_ID='88',ROS_LOCALHOST_ONLY='1')
    if args.lidar_calibration:
        os.environ['SENTRY_ACCEPTANCE_LIDAR_CALIBRATION']=str(args.lidar_calibration.resolve())
    env=dict(os.environ,SENTRY_ACCEPTANCE_OUTPUT=str(out),ROS_LOG_DIR=str(out/'ros_logs'))
    report={'status':'starting','physical_acceptance_passed':False,'stages':[],
            'lidar_calibration':str(args.lidar_calibration) if args.lidar_calibration else None,
            'scope':'Navigation-process startup; fresh local SLAM, not power-cycle or saved-map global localization'}
    def save(): (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    def run(name,command,timeout):
        report['active_stage']=name;save();print('STAGE:',name,flush=True)
        child=subprocess.Popen(command,env=env)
        try:code=child.wait(timeout=timeout)
        except (subprocess.TimeoutExpired,KeyboardInterrupt):
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
            raise
        report['stages'].append({'name':name,'exit_code':code});save()
        if code:raise RuntimeError(name+' failed; subsequent movement skipped')
    launch=None
    save()
    try:
        with (out/'launch.log').open('w') as log:
            launch=subprocess.Popen(['ros2','launch',str(folder/'static_launch.py')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        time.sleep(12)
        if launch.poll() is not None:raise RuntimeError('Navigation launch failed')
        _,initial=snapshot();report['initial_pose_map']=initial;save()
        run('scan_two_turns',[sys.executable,str(folder/'motion_probe.py'),'--execute','--axis','gimbal_yaw','--speed','.4','--turns','2'],135)
        rotation=json.loads((out/'motion_probe.json').read_text())
        if rotation.get('startup_turns_completed')!=2:raise RuntimeError('Two turns not completed')
        grid,current=snapshot()
        run('save_scanned_map',['ros2','run','nav2_map_server','map_saver_cli','-f',str(out/'scanned_map'),'--ros-args','-p','save_map_timeout:=10.0'],20)
        goal=(initial[0]+.20*math.cos(initial[2]),initial[1]+.20*math.sin(initial[2]))
        if not path_is_free(grid,current[:2],goal):raise RuntimeError('Forward map goal corridor occupied or unknown')
        report['goal_map']=goal
        landmarks=out/'landmarks.json'
        landmarks.write_text(json.dumps({'frame':'map','session':out.name,'targets':[{'name':'startup_forward','x':goal[0],'y':goal[1]}]},indent=2)+'\n')
        run('navigate_saved_goal',[sys.executable,str(folder/'navigation_trial.py'),'--execute','--landmarks',str(landmarks)],45)
        navigation=json.loads((out/'navigation_trial.json').read_text())
        if not navigation.get('local_navigation_passed'):raise RuntimeError('Goal acceptance failed')
        run('save_final_map',['ros2','run','nav2_map_server','map_saver_cli','-f',str(out/'map'),'--ros-args','-p','save_map_timeout:=10.0'],20)
        report.update(status='local_startup_task_passed',gimbal_scan_travel_rad=rotation.get('scan_travel_rad'),gimbal_scan_coverage_bins=rotation.get('scan_coverage_bins'),waypoints=navigation['waypoints'],final_wheel_speeds=navigation['final_wheel_speeds'])
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='incomplete',reason=str(exc) or 'Interrupted')
    finally:
        if launch and launch.poll() is None and not (out/'scanned_map.yaml').exists():
            try:
                run('save_partial_map',['ros2','run','nav2_map_server','map_saver_cli','-f',str(out/'partial_map'),'--ros-args','-p','save_map_timeout:=10.0'],20)
            except Exception as exc:report['partial_map_error']=str(exc)
        if launch:
            for sig,seconds in [(signal.SIGINT,15),(signal.SIGTERM,5),(signal.SIGKILL,5)]:
                try:os.killpg(launch.pid,sig)
                except ProcessLookupError:break
                try:launch.wait(timeout=seconds)
                except subprocess.TimeoutExpired:continue
                try:os.killpg(launch.pid,0)
                except ProcessLookupError:break
        report.pop('active_stage',None);save();print('REPORT:',out/'summary.json',flush=True)
    return 0 if report['status']=='local_startup_task_passed' else 1


if __name__=='__main__':raise SystemExit(main())
