#!/usr/bin/env python3
"""Require closed rover patrol loops, no skipped stops, and a measured return home."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from embertrace_interfaces.msg import MissionState, FireAlert
from embertrace_interfaces.srv import OperatorCommand
from nav_msgs.msg import Odometry
from mission_clearance import TreeClearance
from std_msgs.msg import String, UInt32


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--loops', type=int, default=2)
    parser.add_argument('--max-duration', type=float, default=1200)
    parser.add_argument('--wall-timeout', type=float, default=1800)
    parser.add_argument('--output', default='log/patrol-final')
    parser.add_argument('--world-file', choices=['embertrace_forest.sdf', 'embertrace_forest_dense.sdf'], default='embertrace_forest.sdf')
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    if args.loops < 1: parser.error('At least one loop is required')
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    if (output / 'result.json').exists(): raise FileExistsError('Use a fresh output directory')
    route = yaml.safe_load((Path(get_package_share_directory('embertrace_bringup')) / 'config/patrol_waypoints.yaml').read_text())['waypoints']
    sim_root = Path(get_package_share_directory('embertrace_sim'))
    clearance_model = TreeClearance.from_files(sim_root/'worlds'/args.world_file,
                                             sim_root/'config/nav2_params_husky1.yaml')
    base = (route[0]['x'], route[0]['y'])
    expected_visits = len(route) * args.loops + 1
    visit_tolerance = 1.7
    result = {'source_revision': subprocess.check_output(['git', 'log', '-1', '--format=%H', '--', 'src'], text=True).strip(),
              'harness_revision': subprocess.check_output(['git', 'log', '-1', '--format=%H', '--', 'scripts/ros_patrol_trials.py'], text=True).strip(),
              'world_file': args.world_file, 'gazebo_gui': args.gui,
              'visited_waypoints': [], 'visit_tolerance_m': visit_tolerance,
              'closed_loops': 0, 'skipped_waypoints': 0, 'navigation_failures': 0, 'path_length_m': 0., 'false_alerts': 0, 'passed': False,
              'minimum_tree_clearance_m': None, 'closest_tree': None}
    rclpy.init(); node = Node('patrol_acceptance')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    current = {'state': None, 'pose': None}
    def odometry(msg):
        p = msg.pose.pose.position; pose = (p.x, p.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        clearance, tree = clearance_model.closest(p.x, p.y, yaw)
        if clearance is not None and (result['minimum_tree_clearance_m'] is None or
                                     clearance < result['minimum_tree_clearance_m']):
            result.update(minimum_tree_clearance_m=clearance, closest_tree=tree)
        if current['pose'] is not None:
            distance = math.dist(pose, current['pose'])
            if distance < 2: result['path_length_m'] += distance
        current['pose'] = pose
        if len(result['visited_waypoints']) < expected_visits:
            wanted = route[len(result['visited_waypoints']) % len(route)]
            if math.dist(pose, (wanted['x'], wanted['y'])) <= visit_tolerance:
                result['visited_waypoints'].append(wanted['name'])
    def navigation(msg):
        if msg.data in ('ABORTED', 'REJECTED', 'OUT_OF_BOUNDS'): result['navigation_failures'] += 1
    subscriptions = [
        node.create_subscription(UInt32, '/husky1/patrol_loops',
            lambda m: result.update(closed_loops=m.data), 10),
        node.create_subscription(MissionState, '/embertrace/mission_state', lambda m: current.update(state=m.state_name),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Odometry, '/husky1/odometry', odometry, qos_profile_sensor_data),
        node.create_subscription(FireAlert, '/embertrace/fire_alert', lambda m: result.update(false_alerts=result['false_alerts'] + int(m.confirmed)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(String, '/embertrace/navigation_status', navigation, 10),
    ]
    client = node.create_client(OperatorCommand, '/embertrace/operator_command')
    def command(text):
        request = OperatorCommand.Request(); request.command = text
        future = client.call_async(request); rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and future.result().accepted, f'{text} rejected'
    environment = dict(os.environ, IGN_PARTITION=f'embertrace_patrol_{os.getpid()}')
    environment['GZ_PARTITION'] = environment['IGN_PARTITION']
    with (output / 'launch.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'embertrace_bringup', 'embertrace_demo.launch.py',
            'mission_platform:=rover', 'scenario:=no_fire', 'gazebo_gui:='+str(args.gui).lower(),
            'world_file:='+args.world_file, 'use_dashboard:=false', 'use_rviz:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
        try:
            subprocess.run(['ros2', 'run', 'embertrace_core', 'demo_preflight.py', '--platform', 'rover', '--timeout', '120'],
                stdout=log, stderr=subprocess.STDOUT, check=True, timeout=140)
            assert client.wait_for_service(timeout_sec=15)
            command('START'); started = node.get_clock().now().nanoseconds * 1e-9
            wall = time.monotonic(); last_report = 0; pending = ''; wrapped = 0
            with (output / 'launch.log').open() as reader:
                while time.monotonic() - wall < args.wall_timeout:
                    rclpy.spin_once(node, timeout_sec=.1)
                    pending += reader.read(); lines = pending.split('\n'); pending = lines.pop()
                    for line in lines:
                        match = re.search(r'Patrol loop (\d+) complete', line)
                        if match: wrapped = int(match.group(1))
                        if 'Gave up on an unreachable waypoint' in line: result['skipped_waypoints'] += 1
                    elapsed = node.get_clock().now().nanoseconds * 1e-9 - started
                    if time.monotonic() - last_report > 15:
                        print(json.dumps({'elapsed': elapsed, **current, **result}), flush=True); last_report = time.monotonic()
                    if current['state'] in ('PAUSED', 'EMERGENCY_STOP') or result['false_alerts']:
                        raise RuntimeError(f'Unexpected mission state during no-fire patrol: {current}')
                    if result['skipped_waypoints']:
                        raise RuntimeError('A required patrol waypoint was skipped')
                    if result['minimum_tree_clearance_m'] is not None and result['minimum_tree_clearance_m'] < 0:
                        raise RuntimeError('Measured rover footprint overlaps trunk '+str(result['closest_tree']))
                    if result['closed_loops'] >= args.loops: break
                    if elapsed > args.max_duration or process.poll() is not None: raise RuntimeError('Patrol mission deadline or launch exit')
                assert result['closed_loops'] >= args.loops, 'Patrol loops did not complete'
            # The mission policy automatically returns after its configured two loops.
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline and (current['state'] != 'IDLE' or len(result['visited_waypoints']) < expected_visits):
                rclpy.spin_once(node, timeout_sec=.1)
                if result['minimum_tree_clearance_m'] is not None and result['minimum_tree_clearance_m'] < 0:
                    raise RuntimeError('Measured rover footprint overlaps trunk '+str(result['closest_tree']))
            assert current['state'] == 'IDLE' and current['pose'] is not None, 'Patrol did not park in IDLE'
            assert len(result['visited_waypoints']) == expected_visits, 'Route points were not physically visited in order'
            result['return_error_m'] = math.dist(current['pose'], base)
            assert result['return_error_m'] < 1.7 and result['skipped_waypoints'] == 0 and result['false_alerts'] == 0
            result['mission_time_s'] = node.get_clock().now().nanoseconds * 1e-9 - started
            result['passed'] = True
        except Exception as exc:
            result['failure'] = str(exc)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            node.destroy_node(); rclpy.shutdown()
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
