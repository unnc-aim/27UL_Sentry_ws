"""Geometry regression only; does not initialize ROS or send commands."""
import math
from yaw_feedback import encoder_yaw

zero = 18032
assert encoder_yaw(zero, zero) == 0
assert abs(encoder_yaw(0, zero)-encoder_yaw(65535, zero)) < 1e-4
for encoder in [0, 12000, zero, 30000, 54310, 65535]:
    angle = encoder_yaw(encoder, zero)
    assert -math.pi-1e-4 <= angle <= math.pi+1e-4
    # Existing Hub getter is -angle; its velocity conversion applies R(-getter).
    theta = -angle
    for x, y in [(1., 0.), (0., 1.), (.3, -.2)]:
        hub = (x*math.cos(theta)+y*math.sin(theta), -x*math.sin(theta)+y*math.cos(theta))
        tf = (x*math.cos(angle)-y*math.sin(angle), x*math.sin(angle)+y*math.cos(angle))
        assert math.dist(hub, tf) < 1e-12
print('PASS: encoder zero, wrap and existing Hub/TF rotation convention')

# Complete planar navigation conversion, including the existing Hub sign contract.
# Physical lidar now belongs to gimbal_yaw. Body and gimbal headings can differ.
def rotate(v, angle):
    return (v[0]*math.cos(angle)-v[1]*math.sin(angle),
            v[0]*math.sin(angle)+v[1]*math.cos(angle))
for body in (-2.7, 0., 1.4):
    for joint in (-2.2, .4, 2.8):
        desired=(.07, -.12)
        gimbal_velocity=rotate(desired, -(body+joint))
        bridge=tuple(-v for v in gimbal_velocity)
        hub=rotate(tuple(-v for v in bridge),joint)
        world=rotate(hub,body)
        assert math.dist(world,desired)<1e-12
print('PASS: navigation adapter cancels existing Hub sign across body/gimbal headings')

# Exercise the actual production launch without starting any nodes.
from pathlib import Path
import runpy
import xml.etree.ElementTree as ET
from launch import LaunchContext
from launch.actions import LogInfo

workspace = Path(__file__).resolve().parents[2]
launch_path = workspace / 'src/pb2025_robot_description/launch/robot_description_launch.py'
production = runpy.run_path(str(launch_path))
created = []
def record_node(**kwargs):
    created.append(kwargs)
    return LogInfo(msg='Geometry test; no node started')
production['launch_setup'].__globals__['Node'] = record_node

for robot_name, sim_time, real_sentry in [
    ('pb2025_sentry_robot', 'False', True),
    ('pb2025_sentry_robot', '0', True),
    ('pb2025_sentry_robot', 'True', False),
    ('pb2025_infantry_robot', 'False', False),
    ('simulation_robot', 'True', False),
]:
    context = LaunchContext()
    context.launch_configurations.update(
        namespace='', use_sim_time=sim_time, robot_name=robot_name,
        robot_xmacro_file=str(launch_path.parent.parent / 'resource/xmacro' / (robot_name + '.sdf.xmacro')),
        params_file=str(launch_path.parent.parent / 'params/robot_description.yaml'),
        rviz_config_file='', use_rviz='False', use_respawn='False', log_level='info')
    created.clear()
    production['launch_setup'](context)
    state = next(node for node in created if node['package'] == 'robot_state_publisher')
    defaults = next(node for node in created if node['package'] == 'joint_state_publisher')
    assert defaults['condition'].evaluate(context) == (not real_sentry)
    urdf = ET.fromstring(state['parameters'][1]['robot_description'])
    lidar = urdf.find("joint[@name='front_livox_joint']")
    if real_sentry:
        assert urdf.find("joint[@name='gimbal_yaw_odom_joint']") is None
        yaw = urdf.find("joint[@name='gimbal_yaw_joint']")
        assert yaw.find('parent').get('link') == 'chassis'
        assert lidar.find('parent').get('link') == 'gimbal_yaw'
        yaw_xyz = [float(v) for v in yaw.find('origin').get('xyz').split()]
        lidar_xyz = [float(v) for v in lidar.find('origin').get('xyz').split()]
        assert math.dist([a+b for a,b in zip(yaw_xyz,lidar_xyz)], [.16,0.,.18]) < 1e-12
        assert math.dist([float(v) for v in lidar.find('origin').get('rpy').split()],
                         [-math.pi/4,0.,-math.pi/2]) < 1e-12
    else:
        assert urdf.find("joint[@name='gimbal_yaw_odom_joint']") is not None
        if lidar is not None:
            assert lidar.find('parent').get('link') == 'chassis'
print('PASS: production sentry uses measured yaw; simulation/infantry geometry stays unchanged')
