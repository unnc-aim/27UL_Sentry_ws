#!/usr/bin/env python3
"""Manual mapping and stationary saved-map localization; no automatic gimbal scan."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def stop(child):
    for sig, timeout in [(signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 3)]:
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            break
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        try:
            os.killpg(child.pid, 0)
        except ProcessLookupError:
            break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['map', 'save', 'localize', 'drive'])
    parser.add_argument('--map', type=Path, default=HERE/'maps/manual_test/map.yaml')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--face-motion', action='store_true')
    parser.add_argument('--align-only', action='store_true')
    parser.add_argument('--max-speed',type=float,default=.15)
    args = parser.parse_args()
    path = args.map.resolve()
    if path.suffix != '.yaml':
        parser.error('--map must end in .yaml')
    env = dict(os.environ, ROS_DOMAIN_ID='88', ROS_LOCALHOST_ONLY='1')
    env.pop('SENTRY_ACCEPTANCE_SAVED_MAP', None)
    if not env.get('DISPLAY') and Path('/tmp/.X11-unix/X0').exists():
        env['DISPLAY'] = ':0'
        auth = Path('/run/user/1000/gdm/Xauthority')
        if auth.exists():
            env['XAUTHORITY'] = str(auth)

    if args.mode == 'save':
        if path.exists() or path.with_suffix('.pgm').exists():
            raise RuntimeError('Map already exists; use a new --map path to preserve goal coordinates')
        path.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(path.with_suffix('')),
                        '--free', '0.196', '--occ', '0.65', '--fmt', 'pgm', '--mode', 'trinary',
                        '--ros-args', '-p', 'save_map_timeout:=10.0'], env=env, check=True, timeout=20)
        (path.parent/'manifest.json').write_text(json.dumps({
            'map_yaml': str(path),
            'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
            'method': 'manual remote driving; native map_saver_cli'
        }, indent=2)+'\n')
        print('地图已保存：', path, flush=True)
        return
    if args.mode == 'drive':
        if not args.execute:
            parser.error('drive requires --execute and on-site remote takeover')
        if not path.exists():
            raise RuntimeError('Saved map missing')
        selected = path.parent/'selected_goal.json'
        if Path(json.loads(selected.read_text())['map_yaml']).resolve() != path:
            raise RuntimeError('Selected goal belongs to a different map')
        env['SENTRY_ACCEPTANCE_OUTPUT'] = str(HERE/'results'/time.strftime('stationary_goal_%Y%m%d_%H%M%S'))
        os.environ.update(env)
        extra=(['--face-motion'] if args.face_motion else [])+(['--align-only'] if args.align_only else [])
        os.execv(sys.executable, [sys.executable, str(HERE/'navigation_trial.py'), '--execute',
                                '--saved-goal', '--landmarks', str(selected), '--max-speed',str(args.max_speed), *extra])

    existing = subprocess.run(['pgrep', '-f', '[/]livox_ros_driver2_node|[/]pointlio_mapping'], capture_output=True)
    if existing.returncode == 0:
        raise RuntimeError('Lidar/Point-LIO already running. Stop its owning launch first; no process was killed.')
    os.environ.update(env)
    import rclpy
    rclpy.init()
    node = rclpy.create_node('map_test_preflight', enable_rosout=False, start_parameter_services=False)
    end = time.monotonic()+2
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    others = [name for name in node.get_node_names() if name != node.get_name()]
    node.destroy_node();rclpy.shutdown()
    if others:
        raise RuntimeError('Domain 88 occupied: '+', '.join(others))
    if args.mode == 'localize':
        if not path.exists():
            raise RuntimeError('Save the manually built map first')
        env['SENTRY_ACCEPTANCE_SAVED_MAP'] = str(path)
    elif path.exists():
        raise RuntimeError('Map already exists; use a new --map path')
    out = HERE/'results'/time.strftime(args.mode+'_%Y%m%d_%H%M%S')
    out.mkdir(parents=True)
    children = []
    logs = []
    def start(name, command):
        log = open(out/(name+'.log'), 'w')
        logs.append(log)
        child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        return child
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        start('launch', ['ros2', 'launch', str(HERE/'static_launch.py')])
        if not args.no_rviz:
            start('rviz', ['rviz2', '-d', str(HERE/'select_goal.rviz')])
        print('日志：', out, flush=True)
        if args.mode == 'map':
            print('手动建图已启动。请使用原遥控器低速建图。\n'
                  '完成后先停稳，在另一终端执行 save；看到保存成功后，在此 Ctrl+C。', flush=True)
        else:
            start('localization', [sys.executable, '-s', str(HERE/'localization_probe.py'),
                                  '--globalize', '--seconds', '3600',
                                  '--output', str(out/'localization.json')])
            start('selector', [sys.executable, str(HERE/'select_goal.py'), '--map', str(path)])
            print('AMCL 全局初始化已安排，定位过程保持静止。RViz 的 2D Goal Pose 保存目标。\n'
                  '查看 localization.json 的 latest.candidate_stable，并核对现场位置与朝向。\n'
                  '重新摆放车辆后，结束本次 localize，再重新启动。', flush=True)
        last_ready = None
        while True:
            time.sleep(.5)
            if args.mode == 'localize':
                try:
                    latest = json.loads((out/'localization.json').read_text())['latest']
                    ready = latest['candidate_stable'] and time.time()-latest['time'] < 1.2
                except (OSError,ValueError,KeyError):
                    ready = False
                if ready != last_ready:
                    print('定位已稳定，可在地图选点后执行 drive。' if ready else
                          '等待定位数据稳定，保持原地。', flush=True)
                    last_ready = ready
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError('A workflow process exited; inspect logs: '+str(out))
    except KeyboardInterrupt:
        print('停止本次建图/定位进程。地图只在显式 save 时保存。', flush=True)
    finally:
        for child in reversed(children):
            stop(child)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
