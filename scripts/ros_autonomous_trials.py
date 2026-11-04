#!/usr/bin/env python3
"""Measure autonomous fire investigations using evaluation-only source truth.

Launches fresh seeded Gazebo missions. It never publishes source coordinates or
sensor substitutes. Failed and unresolved trials remain in the summary.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from embertrace_interfaces.msg import MissionState, FireAlert, EnvironmentalReading, PerceptionStatus
from embertrace_interfaces.srv import OperatorCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String


def trial(args, seed, output, truth):
    robot = 'drone1' if args.platform == 'drone' else 'husky1'
    result = {'source_revision': args.source_revision, 'seed': seed, 'platform': args.platform, 'confirmed': False, 'resolved': False,
              'source_error_m': None, 'path_length_m': 0., 'states': [], 'navigation_failures': 0}
    environment = dict(os.environ, IGN_PARTITION=f'embertrace_trial_{os.getpid()}_{seed}')
    environment['GZ_PARTITION'] = environment['IGN_PARTITION']
    rclpy.init(); node = Node('autonomous_trial')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    state = {'alert': None, 'pose': None}
    telemetry = {}
    def mission(msg):
        if not result['states'] or result['states'][-1] != msg.state_name:
            result['states'].append(msg.state_name)
    def odometry(msg):
        p = msg.pose.pose.position
        pose = (p.x, p.y)
        if state['pose'] is not None:
            distance = math.dist(pose, state['pose'])
            if distance < 2.: result['path_length_m'] += distance
        state['pose'] = pose
    def navigation(msg):
        if msg.data in ('ABORTED', 'REJECTED', 'OUT_OF_BOUNDS'):
            result['navigation_failures'] += 1
    subscriptions = [
        node.create_subscription(MissionState, '/embertrace/mission_state', mission,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(FireAlert, '/embertrace/fire_alert', lambda msg: state.update(alert=msg),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Odometry, f'/{robot}/odometry', odometry, qos_profile_sensor_data),
        node.create_subscription(String, '/embertrace/navigation_status', navigation, 10),
        node.create_subscription(PoseStamped, '/embertrace/navigation_goal', lambda m: telemetry.update(
            goal=[m.pose.position.x, m.pose.position.y, m.header.frame_id]), 10),
        node.create_subscription(PoseWithCovarianceStamped, '/embertrace/source_estimate', lambda m: telemetry.update(
            estimate=[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.covariance[0], m.pose.covariance[7]]), 10),
        node.create_subscription(Twist, f'/{robot}/cmd_vel', lambda m: telemetry.update(
            command=[m.linear.x, m.linear.y, m.linear.z]), 10),
        node.create_subscription(EnvironmentalReading, '/drone1/environment' if robot == 'drone1' else '/embertrace/environment',
            lambda m: telemetry.update(environment=[m.smoke_ppm, m.temperature_c, m.fused_confidence,
                                                    m.smoke_detected, m.temperature_anomaly]), qos_profile_sensor_data),
        node.create_subscription(PerceptionStatus, '/drone1/perception_status' if robot == 'drone1' else '/embertrace/perception_status',
            lambda m: telemetry.update(perception=[m.environment_fresh, m.thermal_fresh, m.wind_condition]), 10),
    ]
    client = node.create_client(OperatorCommand, '/embertrace/operator_command')
    with (output / f'{args.platform}_{seed}.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'embertrace_bringup', 'embertrace_demo.launch.py',
            f'mission_platform:={args.platform}', 'scenario:=fire', 'gazebo_gui:=false',
            'use_rviz:=false', 'use_dashboard:=false', f'random_seed:={seed}',
            f'drone_random_seed:={seed + 1}'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True, env=environment)
        try:
            subprocess.run(['ros2', 'run', 'embertrace_core', 'demo_preflight.py',
                '--platform', args.platform, '--timeout', '120'], check=True, timeout=140,
                stdout=log, stderr=subprocess.STDOUT)
            assert client.wait_for_service(timeout_sec=15), 'Operator service unavailable'
            # Discover the trial's odometry before starting its path-length metric.
            until = time.monotonic() + 15
            while state['pose'] is None and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
            request = OperatorCommand.Request(); request.command = 'START'
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done() and future.result().accepted, 'START rejected'
            started = node.get_clock().now().nanoseconds * 1e-9
            wall = time.monotonic()
            last_report = 0
            print('Search subscriptions: ' + str(node.get_subscriber_names_and_types_by_node('search_planner_node', '/')), flush=True)
            while time.monotonic() - wall < args.wall_timeout:
                rclpy.spin_once(node, timeout_sec=.1)
                elapsed = node.get_clock().now().nanoseconds * 1e-9 - started
                if time.monotonic() - last_report >= 5:
                    print(json.dumps({'seed': seed, 'elapsed': round(elapsed, 2), 'pose': state['pose'],
                        'state': result['states'][-1] if result['states'] else None, **telemetry}), flush=True)
                    last_report = time.monotonic()
                if state['alert'] is not None or elapsed > args.max_duration:
                    break
                if process.poll() is not None:
                    raise RuntimeError('Launch exited during mission')
            result['mission_time_s'] = node.get_clock().now().nanoseconds * 1e-9 - started
            alert = state['alert']
            if alert is not None:
                result['confirmed'] = alert.confirmed
                result['resolved'] = 'unresolved' not in alert.evidence
                p = alert.estimated_source.pose.position
                result['reported_position'] = [p.x, p.y]
                if result['resolved']:
                    result['source_error_m'] = math.hypot(p.x - truth[0], p.y - truth[1])
            else:
                result['failure'] = 'No alert before mission deadline'
        except Exception as exc:
            result['failure'] = str(exc)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            node.destroy_node(); rclpy.shutdown()
    (output / f'{args.platform}_{seed}.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--platform', choices=('rover', 'drone', 'combined'), required=True)
    parser.add_argument('--trials', type=int, default=10)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--seed-start', type=int, default=142)
    parser.add_argument('--max-duration', type=float, default=600)
    parser.add_argument('--wall-timeout', type=float, default=1200)
    parser.add_argument('--output', default='log/autonomous-trials')
    args = parser.parse_args()
    if args.trials < 1 or args.max_duration <= 0 or args.wall_timeout <= 0:
        parser.error('Trial count and deadlines must be positive')
    output = Path(args.output) / args.platform; output.mkdir(parents=True, exist_ok=True)
    parameters = yaml.safe_load((Path(get_package_share_directory('embertrace_bringup')) /
                                'config/embertrace.yaml').read_text())
    parameters = parameters['environment_simulator_node']['ros__parameters']
    truth = (parameters['source_x'], parameters['source_y'])
    args.source_revision = subprocess.check_output(['git', 'log', '-1', '--format=%H', '--', 'src'], text=True).strip()
    results = []
    for seed in range(args.seed_start, args.seed_start + args.trials):
        previous = output / f'{args.platform}_{seed}.json'
        if previous.exists():
            if not args.resume:
                raise FileExistsError(f'Refusing to overwrite {previous}; use a fresh output or --resume')
            result = json.loads(previous.read_text())
            if result.get('source_revision') != args.source_revision:
                raise RuntimeError('Previous trial used different source; choose a fresh output directory')
            results.append(result)
        else:
            results.append(trial(args, seed, output, truth))
    errors = [row['source_error_m'] for row in results if row['source_error_m'] is not None]
    summary = {'platform': args.platform, 'trials': args.trials,
               'confirmed': sum(row['confirmed'] for row in results),
               'resolved': len(errors), 'median_source_error_m': statistics.median(errors) if errors else None,
               'failures': [row['seed'] for row in results if row.get('failure') or not row['resolved']],
               'results': results}
    summary['passed'] = len(errors) == args.trials and summary['confirmed'] == args.trials and statistics.median(errors) < 2.
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)
    raise SystemExit(0 if summary['passed'] else 1)


if __name__ == '__main__':
    main()
