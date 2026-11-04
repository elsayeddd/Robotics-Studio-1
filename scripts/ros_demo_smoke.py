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
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from embertrace_interfaces.msg import MissionState, ThermalCue
from sensor_msgs.msg import Image
from embertrace_interfaces.srv import OperatorCommand


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--platform', choices=('combined', 'rover', 'drone'), default='combined')
    parser.add_argument('--output', default='log/demo-smoke')
    parser.add_argument('--max-sensor-holds', type=int, default=0,
                        help='Verify stopped motion and operator recovery after at most this many sensor holds')
    args = parser.parse_args()
    if args.max_sensor_holds < 0: parser.error('Sensor hold count must be non-negative')
    robots = ('husky1', 'drone1') if args.platform == 'combined' else (('husky1',) if args.platform == 'rover' else ('drone1',))
    active_robot = 'husky1' if args.platform == 'rover' else 'drone1'
    output = Path(args.output) / args.platform
    output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, IGN_PARTITION=f'embertrace_smoke_{os.getpid()}')
    environment['GZ_PARTITION'] = environment['IGN_PARTITION']
    rclpy.init()
    node = Node('demo_smoke')
    with (output / 'launch.log').open('w') as log:
        process = None
        try:
            poses, commands, state = {}, {}, {'name': None, 'detail': None}
            images, cues, sensor_holds = deque(maxlen=12), deque(maxlen=20), []
            def mission(message):
                previous = state['name']
                state.update(name=message.state_name, detail=message.detail)
                if message.state == MissionState.PAUSED and previous!='PAUSED':
                    state['pause_snapshot'] = dict(wall_s=time.monotonic(),
                        ros_s=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                        images=list(images), cues=list(cues))
            subscriptions = [node.create_subscription(MissionState, '/embertrace/mission_state',
                mission,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
            root = '/embertrace' if active_robot == 'husky1' else '/drone1'
            subscriptions.extend([
                node.create_subscription(Image, '/'+active_robot+'/thermal/image',
                    lambda m: images.append(dict(wall_s=time.monotonic(), stamp_s=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                                                 frame=m.header.frame_id, encoding=m.encoding)), qos_profile_sensor_data),
                node.create_subscription(ThermalCue, root+'/thermal_cue',
                    lambda m: cues.append(dict(wall_s=time.monotonic(), stamp_s=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                                               frame=m.header.frame_id, valid=m.valid)), qos_profile_sensor_data),
            ])
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
            def until(predicate, timeout, recover_holds=False):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    pump(0.1)
                    if recover_holds and state['name'] == 'PAUSED':
                        recover_sensor_hold()
                    if predicate():
                        return
                raise AssertionError(f'Timeout in {args.platform}; state={state}, poses={poses}')
            def command(text, accepted=True, wait_ready=False):
                deadline = time.monotonic() + (15 if wait_ready else 0)
                while True:
                    request = OperatorCommand.Request(); request.command = text
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                    assert future.done() and future.exception() is None, f'Command timed out: {text}'
                    response = future.result()
                    if response.accepted == accepted: return
                    if not wait_ready or time.monotonic() >= deadline:
                        raise AssertionError(f'Command failed: {text}: {response.message}')
                    print(f'Waiting for {text} readiness: {response.message}', flush=True)
                    pump(.5)
            def zero():
                for robot in robots:
                    received, twist = commands.get(robot, (0, None))
                    assert twist is not None and time.monotonic() - received < 0.75, f'Missing {robot} heartbeat'
                    assert sum(abs(v) for v in (twist.linear.x, twist.linear.y, twist.linear.z,
                                                twist.angular.x, twist.angular.y, twist.angular.z)) < 1e-6, f'{robot} still moving'
            def recover_sensor_hold():
                assert len(sensor_holds)<args.max_sensor_holds, f'Unexpected sensor hold: {state}'
                assert state['detail']=='Wind/platform or sensor availability changed; review then RESUME', state
                cause = state['detail']
                snapshot = state.get('pause_snapshot', {})
                pump(1); zero()
                held = dict(poses)
                pump(.5); zero()
                drift = max(math.dist(held[r], poses[r]) for r in robots)
                assert drift<.2, f'Vehicle drifted during sensor hold: {drift}'
                subprocess.run(['ros2', 'run', 'embertrace_core', 'demo_preflight.py',
                    '--platform', args.platform, '--timeout', '20'],
                    env=environment, check=True, timeout=25)
                command('RESUME', wait_ready=True)
                until(lambda: state['name']!='PAUSED', 5)
                last_image = snapshot.get('images', [])[-1:]
                last_cue = snapshot.get('cues', [])[-1:]
                sensor_holds.append(dict(cause=cause, stopped_drift_m=drift,
                    ir_wall_gap_s=snapshot['wall_s']-last_image[0]['wall_s'] if last_image else None,
                    last_ir_cue_valid=last_cue[0]['valid'] if last_cue else None))
                print('Verified sensor stop and operator RESUME: '+json.dumps(sensor_holds[-1]), flush=True)
            # Establish the recorder before the servers, as in the mission
            # campaign, so startup readiness also covers this DDS participant.
            process = subprocess.Popen(['ros2', 'launch', 'embertrace_bringup', 'embertrace_demo.launch.py',
                'use_rviz:=false', 'use_dashboard:=false', 'gazebo_gui:=false',
                f'mission_platform:={args.platform}', 'scenario:=no_fire'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
            subprocess.run(['ros2', 'run', 'embertrace_core', 'demo_preflight.py',
                            '--platform', args.platform, '--timeout', '120'],
                           env=environment, check=True, timeout=140)
            until(lambda: len(poses) == len(robots) and state['name'] is not None
                  and client.service_is_ready(), 25)
            initial = dict(poses)
            command('START', wait_ready=True)
            until(lambda: math.dist(initial[active_robot], poses[active_robot]) > 0.8, 70, recover_holds=True)
            moved = {r: math.dist(initial[r], poses[r]) for r in robots}
            for robot in robots:
                if robot != active_robot:
                    assert moved[robot] < 0.5, f'Unselected {robot} moved'
            command('PAUSE'); pump(1); zero()
            command('RESUME'); until(lambda: state['name'] == 'PATROL', 5)
            command('RETURN'); until(lambda: state['name'] == 'IDLE', 75, recover_holds=True); pump(1); zero()
            for robot in robots:
                assert math.dist(initial[robot], poses[robot]) < 1.7, f'{robot} did not return to its base'
            command('START', wait_ready=True); until(lambda: state['name'] == 'PATROL', 5)
            command('E_STOP'); pump(1); zero()
            for text in ('START', 'RESUME', 'RETURN'):
                command(text, accepted=False)
            command('E_STOP_RELEASE'); until(lambda: state['name'] == 'IDLE', 5); zero()
            result = {'platform': args.platform, 'movement_m': moved, 'controls': 'passed',
                      'return': 'passed', 'emergency_stop': 'passed', 'verified_sensor_holds': sensor_holds}
            (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            print('Gazebo integration passed: ' + json.dumps(result), flush=True)
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            if node:
                node.destroy_node()
                if rclpy.ok(): rclpy.shutdown()
            print(f'Launch evidence: {output / "launch.log"}', flush=True)


if __name__ == '__main__':
    main()
