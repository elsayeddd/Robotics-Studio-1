#!/usr/bin/env python3
"""Record Gazebo sensor trials, tune on training data, report held-out metrics.

These stationary near-source trials exercise both sensor stacks. They measure
local perception, not search success, flight dynamics or real-fire accuracy.
Run with ROS sourced and WSLg, or under xvfb-run with software Mesa.
"""
import argparse
import csv
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

import rclpy
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_prefix

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('trial_recorder',
    ROOT / 'src/embertrace_core/scripts/record_perception.py')
recorder_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder_module)
SCENARIOS = ('fire', 'no_fire', 'dust', 'hot_surface', 'visual_decoy', 'dropout')


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def trial(output, split, scenario, seed, duration):
    name = f'{split}_{scenario}_{seed}'
    paths = {}
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    for robot in ('husky1', 'drone1'):
        paths[robot] = output / f'{name}_{robot}.csv'
        node = recorder_module.Recorder(SimpleNamespace(
            robot=robot, output=str(paths[robot]), trial=name))
        nodes.append(node)
        executor.add_node(node)
    with (output / f'{name}.log').open('w') as log:
        trial_env = dict(os.environ)
        # Gazebo transport is separate from DDS. Fresh partitions prevent a
        # stopped trial's discovery data from interfering with the next world.
        trial_env['IGN_PARTITION'] = f'embertrace_{name}_{os.getpid()}'
        trial_env['GZ_PARTITION'] = trial_env['IGN_PARTITION']
        # Explicit spawn poses belong to this test fixture, never to autonomy.
        process = subprocess.Popen([
            'ros2', 'launch', 'embertrace_bringup', 'embertrace_demo.launch.py',
            'use_rviz:=false', 'use_dashboard:=false', 'use_nav2:=false',
            'gazebo_gui:=false',
            'use_drone:=true', f'scenario:={scenario}', f'random_seed:={seed}',
            f'drone_random_seed:={seed + 1}',
            'rover_x:=10.5', 'rover_y:=5.5', 'drone_x:=12.0', 'drone_y:=7.0',
            'dropout_start_s:=4.0', 'dropout_duration_s:=4.0'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=trial_env)
        first_stamp = None
        deadline = time.monotonic() + 90
        try:
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'{name}: launch exited; see {log.name}')
                executor.spin_once(timeout_sec=0.05)
                if all(node.rows > 0 for node in nodes):
                    now_s = nodes[0].get_clock().now().nanoseconds * 1e-9
                    if first_stamp is None:
                        first_stamp = now_s
                    if now_s - first_stamp >= duration:
                        break
            else:
                raise RuntimeError(f'{name}: incomplete trial after 90 s; '
                                   f'rows={[node.rows for node in nodes]}, '
                                   f'clock={nodes[0].get_clock().now().nanoseconds * 1e-9}, '
                                   f'first_stamp={first_stamp}; see {log.name}')
        finally:
            for node in nodes:
                node.drain(force=True)
                node.csv_handle.close()
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            stop(process)
    for robot, path in paths.items():
        rows = list(csv.DictReader(path.open()))
        if len(rows) < 30 or sum(int(row['environment_available']) for row in rows) < 20:
            raise RuntimeError(f'{name}/{robot}: insufficient recorded samples')
        if scenario == 'dropout' and not any(row['environment_available'] == '0' for row in rows):
            raise RuntimeError(f'{name}/{robot}: dropout was not recorded')
        if scenario == 'visual_decoy' and robot == 'husky1' and not any(
                float(row['visual_confidence']) >= 0.55 for row in rows):
            raise RuntimeError(f'{name}: visual decoy was not visible to the rover camera')
    print(f'Recorded {name}: both vehicle sensor streams', flush=True)
    return paths


def combine(paths, destination):
    with destination.open('w', newline='') as output:
        writer = csv.writer(output)
        for index, path in enumerate(paths):
            with path.open(newline='') as source:
                reader = csv.reader(source)
                header = next(reader)
                if index == 0:
                    writer.writerow(header)
                writer.writerows(reader)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='log/gazebo-perception')
    parser.add_argument('--duration', type=float, default=12)
    args = parser.parse_args()
    if args.duration < 10:
        parser.error('duration must cover warmup, dropout and recovery (at least 10 s)')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    files = {(split, robot): [] for split in ('training', 'heldout') for robot in ('husky1', 'drone1')}
    for split, seed in [('training', 42), ('heldout', 142)]:
        for scenario in SCENARIOS:
            paths = trial(output, split, scenario, seed, args.duration)
            for robot, path in paths.items():
                files[split, robot].append(path)
    for (split, robot), paths in files.items():
        combine(paths, output / f'{split}_{robot}.csv')
    evaluator = str(Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/evaluate_perception')
    subprocess.run(['python3', str(ROOT / 'scripts/tune_perception.py'),
                    str(output / 'training_husky1.csv'), '--evaluator', evaluator,
                    '--output', str(output / 'threshold_sweep.csv')], check=True)
    candidate = next(csv.DictReader((output / 'threshold_sweep.csv').open()))
    options = ['--alpha', candidate['alpha'], '--smoke-enter', candidate['smoke_enter'],
               '--heat-enter', candidate['heat_enter']]
    (output / 'selected_parameters.json').write_text(json.dumps(candidate, indent=2) + '\n')
    for robot in ('husky1', 'drone1'):
        for label, extra in [('defaults', []), ('selected', options)]:
            result = subprocess.check_output([evaluator, str(output / f'heldout_{robot}.csv'), *extra], text=True)
            (output / f'metrics_{robot}_{label}.csv').write_text(result)
            rows = list(csv.DictReader(io.StringIO(result)))
            if len(rows) != len(SCENARIOS):
                raise RuntimeError('Missing scenario metrics')
            if robot == 'husky1' and label == 'defaults':
                for row in rows:
                    if row['scenario'] in ('fire', 'dropout'):
                        if int(row['detected_events']) < 1:
                            raise RuntimeError('Rover did not detect ' + row['scenario'])
                    elif int(row['fp']) != 0:
                        raise RuntimeError('False rover confirmation in ' + row['scenario'])
            print(f'{robot} / {label}\n{result}', flush=True)
    print(f'Gazebo perception campaign passed. Recordings and metrics: {output}', flush=True)


if __name__ == '__main__':
    main()
