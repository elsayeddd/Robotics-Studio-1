#!/usr/bin/env python3
"""Check real Gazebo motion, mission controls and return in every supported mode."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from embertrace_interfaces.msg import MissionState
from embertrace_interfaces.srv import OperatorCommand


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--platform', choices=('combined', 'rover', 'drone'), default='combined')
    parser.add_argument('--output', default='log/demo-smoke')
    args = parser.parse_args()
    robots = ('husky1', 'drone1') if args.platform == 'combined' else (('husky1',) if args.platform == 'rover' else ('drone1',))
    output = Path(args.output) / args.platform
    output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, IGN_PARTITION=f'embertrace_smoke_{os.getpid()}')
    environment['GZ_PARTITION'] = environment['IGN_PARTITION']
    with (output / 'launch.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'embertrace_bringup', 'embertrace_demo.launch.py',
            'use_rviz:=false', 'use_dashboard:=false', 'gazebo_gui:=false',
            f'mission_platform:={args.platform}', 'scenario:=no_fire'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
        node = None
        try:
            subprocess.run(['ros2', 'run', 'embertrace_core', 'demo_preflight.py',
                            '--platform', args.platform, '--timeout', '120'], check=True, timeout=140)
            rclpy.init()
            node = Node('demo_smoke')
            poses, commands, state = {}, {}, {'name': None}
            subscriptions = [node.create_subscription(MissionState, '/embertrace/mission_state',
                lambda msg: state.update(name=msg.state_name),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
            def record(robot, msg):
                p = msg.pose.pose.position
                poses[robot] = (p.x, p.y, p.z)
            for robot in robots:
                subscriptions.append(node.create_subscription(Odometry, '/' + robot + '/odometry',
                    lambda msg, robot=robot: record(robot, msg), qos_profile_sensor_data))
                subscriptions.append(node.create_subscription(Twist, '/' + robot + '/cmd_vel',
                    lambda msg, robot=robot: commands.update({robot: (time.monotonic(), msg)}), 10))
            client = node.create_client(OperatorCommand, '/embertrace/operator_command')
            def pump(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
            def until(predicate, timeout):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    pump(0.1)
                    if predicate():
                        return
                raise AssertionError(f'Timeout in {args.platform}; state={state}, poses={poses}')
            def command(text, accepted=True):
                request = OperatorCommand.Request(); request.command = text
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().accepted == accepted, f'Command failed: {text}'
            def zero():
                for robot in robots:
                    received, twist = commands.get(robot, (0, None))
                    assert twist is not None and time.monotonic() - received < 0.75, f'Missing {robot} heartbeat'
                    assert sum(abs(v) for v in (twist.linear.x, twist.linear.y, twist.linear.z,
                                                twist.angular.x, twist.angular.y, twist.angular.z)) < 1e-6, f'{robot} still moving'
            until(lambda: len(poses) == len(robots) and client.service_is_ready(), 25)
            initial = dict(poses)
            command('START')
            until(lambda: all(math.dist(initial[r], poses[r]) > 0.8 for r in robots), 70)
            moved = {r: math.dist(initial[r], poses[r]) for r in robots}
            command('PAUSE'); pump(1); zero()
            command('RESUME'); until(lambda: state['name'] == 'PATROL', 5)
            command('RETURN'); until(lambda: state['name'] == 'IDLE', 75); pump(1); zero()
            for robot in robots:
                assert math.dist(initial[robot], poses[robot]) < 1.7, f'{robot} did not return to its base'
            command('START'); until(lambda: state['name'] == 'PATROL', 5)
            command('E_STOP'); pump(1); zero()
            for text in ('START', 'RESUME', 'RETURN'):
                command(text, accepted=False)
            command('E_STOP_RELEASE'); until(lambda: state['name'] == 'IDLE', 5); zero()
            result = {'platform': args.platform, 'movement_m': moved, 'controls': 'passed',
                      'return': 'passed', 'emergency_stop': 'passed'}
            (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            print('Gazebo integration passed: ' + json.dumps(result), flush=True)
        finally:
            if node:
                node.destroy_node(); rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            print(f'Launch evidence: {output / "launch.log"}', flush=True)


if __name__ == '__main__':
    main()
