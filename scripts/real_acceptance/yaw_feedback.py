#!/usr/bin/env python3
"""One-way measured yaw: live domain 0 read-only -> diagnostic domain 88 joints.

No publisher, parameter services or rosout in the live domain. This is not a
control bridge. The angle matches the existing Hub's R(-theta) convention;
physical axis calibration remains a separate acceptance step.
"""
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from custom_msgs.msg import ReadLkMotor
from sensor_msgs.msg import JointState
import yaml


def encoder_yaw(encoder, zero):
    delta = int(encoder) - int(zero)
    if delta > 32767:
        delta -= 65536
    if delta < -32768:
        delta += 65536
    return delta / 65535.0 * 2 * math.pi


def main():
    config = yaml.safe_load((Path(__file__).resolve().parents[2] /
        'src/universal_controller/config/controller_params.sentry.yaml').read_text())
    # Locate the existing chassis configuration without changing the controller.
    def chassis(tree):
        if isinstance(tree, dict):
            if 'yaw_center_ecd' in tree and 'topic_yaw_read' in tree:
                return tree
            for value in tree.values():
                found = chassis(value)
                if found:
                    return found
        return None
    settings = chassis(config)
    if not settings:
        raise RuntimeError('Existing yaw topic/zero configuration not found')
    live, test = Context(), Context()
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    live.init(domain_id=0)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    test.init(domain_id=88)
    reader = rclpy.create_node('acceptance_yaw_reader', context=live,
                              enable_rosout=False, start_parameter_services=False)
    writer = rclpy.create_node('acceptance_yaw_feedback', context=test,
                              enable_rosout=False, start_parameter_services=False)
    publisher = writer.create_publisher(JointState, '/joint_states', 10)
    last = [None, 0.0]
    def receive(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = reader.get_clock().now().nanoseconds * 1e-9 - stamp
        if msg.online and 0 <= age < 0.2:
            last[:] = [msg, time.monotonic()]
        else:
            last[0] = None
    reader.create_subscription(ReadLkMotor, settings['topic_yaw_read'], receive, qos_profile_sensor_data)
    def publish():
        msg, arrival = last
        if msg is None or time.monotonic() - arrival > .2:
            return
        out = JointState()
        # Keep measurement time: do not restamp stale feedback as fresh.
        out.header.stamp = msg.header.stamp
        out.name = ['gimbal_yaw_joint']
        out.position = [encoder_yaw(msg.encoder, settings['yaw_center_ecd'])]
        publisher.publish(out)
    reader.create_timer(.02, publish)
    executor = SingleThreadedExecutor(context=live)
    executor.add_node(reader)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        reader.destroy_node(); writer.destroy_node()
        live.try_shutdown(); test.try_shutdown()


if __name__ == '__main__':
    main()
