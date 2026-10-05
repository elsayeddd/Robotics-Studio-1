#!/usr/bin/env python3
"""Exercise real ROS transport, mission confirmation, dropout and E-stop latching."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from embertrace_interfaces.msg import EnvironmentalReading, MissionState, FireAlert
from embertrace_interfaces.srv import OperatorCommand
from sensor_msgs.msg import Image


def run():
    rclpy.init()
    node = Node('perception_integration_test')
    raw = node.create_publisher(EnvironmentalReading, '/embertrace/raw_environment', qos_profile_sensor_data)
    camera = node.create_publisher(Image, '/husky1/camera/image', qos_profile_sensor_data)
    state = {'value': None, 'alerts': 0, 'env': None}
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(MissionState, '/embertrace/mission_state', lambda m: state.update(value=m.state_name), transient),
        node.create_subscription(FireAlert, '/embertrace/fire_alert', lambda m: state.update(alerts=state['alerts'] + int(m.confirmed)), transient),
        node.create_subscription(EnvironmentalReading, '/embertrace/environment', lambda m: state.update(env=m), qos_profile_sensor_data),
    ]
    client = node.create_client(OperatorCommand, '/embertrace/operator_command')

    def pump(duration, smoke=None, heat=22, visual=True):
        end = time.monotonic() + duration
        next_sample = 0
        while time.monotonic() < end:
            if smoke is not None and time.monotonic() >= next_sample:
                stamp = node.get_clock().now().to_msg()
                reading = EnvironmentalReading()
                reading.header.stamp = stamp
                reading.header.frame_id = 'husky1_odom'
                reading.sample_pose.header = reading.header
                reading.sample_pose.pose.orientation.w = 1.0
                reading.smoke_ppm = float(smoke)
                reading.temperature_c = float(heat)
                reading.wind_speed_mps = 2.5
                raw.publish(reading)
                if visual:
                    image = Image()
                    image.header.stamp = stamp
                    image.header.frame_id = 'husky1_camera_link'
                    image.height = image.width = 32
                    image.step = 96
                    image.encoding = 'rgb8'
                    image.data = bytes([255, 100, 10] * 32 * 32)
                    camera.publish(image)
                next_sample = time.monotonic() + 0.2
            rclpy.spin_once(node, timeout_sec=0.02)

    def command(text):
        request = OperatorCommand.Request()
        request.command = text
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done():
            raise RuntimeError('Operator command timed out: ' + text)
        return future.result().accepted

    logs = tempfile.TemporaryDirectory(prefix='embertrace_ros_test_')
    processes, handles = [], []
    try:
        for executable in ('perception_node', 'camera_verifier_node', 'mission_manager_node'):
            handle = (Path(logs.name) / (executable + '.log')).open('w')
            handles.append(handle)
            processes.append(subprocess.Popen(['ros2', 'run', 'embertrace_core', executable],
                                             stdout=handle, stderr=subprocess.STDOUT, start_new_session=True))
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Mission service did not start')
        pump(1.2, 1)
        assert command('START')
        pump(3, 1)
        assert state['value'] == 'PATROL' and state['alerts'] == 0, 'Vision-only alarm'
        assert state['env'] is not None and state['env'].fused_confidence == 0, 'Colour bypassed fusion gate'
        pump(3, 60, 22, visual=False)
        assert state['alerts'] == 0, 'Dust or expired camera confirmed fire'
        pump(5, 60, 80)
        assert state['value'] == 'ALERT' and state['alerts'] == 1, 'Multi-sensor fire did not alert'
        assert command('E_STOP')
        assert not command('START'), 'START bypassed E-stop latch'
        assert command('RESET')
        pump(1.0, 1)
        assert command('START')
        pump(3.0)
        assert state['value'] == 'PAUSED', 'Environmental dropout did not pause mission'
        print('ROS perception/mission integration passed')
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
        for handle in handles:
            handle.close()
        for path in Path(logs.name).glob('*.log'):
            print(path.name + '\n' + path.read_text()[-3000:])
        logs.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
