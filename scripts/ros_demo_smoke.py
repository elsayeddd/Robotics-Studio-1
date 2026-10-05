#!/usr/bin/env python3
"""Launch the complete environment, check every sensor, motion and E-stop.

Run with ROS sourced and a display (WSLg or xvfb-run). This is a short integration
check, not the ten-trial source localisation/perception acceptance campaign.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from embertrace_interfaces.srv import OperatorCommand


def main():
    output = Path('log/demo-smoke')
    output.mkdir(parents=True, exist_ok=True)
    with (output / 'launch.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'embertrace_bringup', 'embertrace_demo.launch.py',
                                    'use_rviz:=false', 'use_dashboard:=false', 'use_drone:=true',
                                    'use_nav2:=true', 'scenario:=no_fire'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = None
        try:
            preflight = subprocess.run(['ros2', 'run', 'embertrace_core', 'demo_preflight.py',
                                        '--timeout', '90'], timeout=100)
            if preflight.returncode:
                raise RuntimeError('Preflight failed: see log/demo-smoke/launch.log')
            rclpy.init()
            node = Node('demo_smoke')
            poses = {}
            commands = {}
            def record(robot, msg):
                p = msg.pose.pose.position
                poses[robot] = (p.x, p.y, p.z)
            subscriptions = []
            for robot in ('husky1', 'drone1'):
                subscriptions.append(node.create_subscription(Odometry, '/' + robot + '/odometry',
                    lambda msg, robot=robot: record(robot, msg), qos_profile_sensor_data))
                subscriptions.append(node.create_subscription(Twist, '/' + robot + '/cmd_vel',
                    lambda msg, robot=robot: commands.update({robot: (time.monotonic(), msg)}), 10))
            client = node.create_client(OperatorCommand, '/embertrace/operator_command')
            def pump(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
            def command(text):
                request = OperatorCommand.Request()
                request.command = text
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                if not future.done() or not future.result().accepted:
                    raise RuntimeError('Command failed: ' + text)
            # DDS discovery and the first sensor sample are asynchronous. Wait
            # for the actual streams instead of assuming a two-second startup.
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline and (len(poses) < 2 or not client.service_is_ready()):
                pump(0.1)
            assert len(poses) == 2, 'Both odometry streams are required'
            initial = dict(poses)
            command('START')
            deadline = time.monotonic() + 45
            moved = {}
            while time.monotonic() < deadline:
                pump(0.2)
                moved = {r: math.dist(initial[r], poses[r]) for r in initial}
                if all(d > 0.5 for d in moved.values()):
                    break
            assert all(d > 0.5 for d in moved.values()), 'Vehicle did not move: ' + str(moved)
            command('E_STOP')
            pump(1)
            for robot in initial:
                received, twist = commands.get(robot, (0, None))
                assert twist is not None and time.monotonic() - received < 0.5, 'Missing command heartbeat'
                assert abs(twist.linear.x) + abs(twist.linear.y) + abs(twist.linear.z) + abs(twist.angular.z) < 1e-6, 'E-stop did not zero ' + robot
            print('Gazebo sensor, rover/drone motion and E-stop smoke check passed: ' + str(moved))
        finally:
            if node:
                node.destroy_node()
                rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
            content = (output / 'launch.log').read_text()
            errors = [line for line in content.splitlines() if '[ERROR]' in line or '[FATAL]' in line]
            print('\n'.join(errors[:25]))
            print(content[-8000:])


if __name__ == '__main__':
    main()
