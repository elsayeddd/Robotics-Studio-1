#!/usr/bin/env python3
"""Check real Fortress scene changes and their coupling to local sensor streams."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from embertrace_interfaces.msg import EnvironmentalReading, FireState, MissionState
from embertrace_interfaces.srv import SetFireState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='log/scene-integration')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    environment = dict(os.environ, IGN_PARTITION=f'embertrace_scene_{os.getpid()}')
    environment['GZ_PARTITION'] = environment['IGN_PARTITION']
    rclpy.init()
    node = Node('scene_integration_test')
    values = {'fire': None, 'mission': None, 'readings': []}
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(FireState, '/embertrace/fire_state',
            lambda msg: values.update(fire=msg), transient),
        node.create_subscription(MissionState, '/embertrace/mission_state',
            lambda msg: values.update(mission=msg), transient),
        node.create_subscription(EnvironmentalReading, '/drone1/environment',
            lambda msg: values['readings'].append((time.monotonic(), msg)), qos_profile_sensor_data),
    ]
    client = node.create_client(SetFireState, '/embertrace/set_fire_state')

    def until(predicate, timeout=90):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise AssertionError('Scene/sensor readiness deadline exceeded')

    result = {}
    with (output / 'launch.log').open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'embertrace_bringup', 'embertrace_demo.launch.py',
            'mission_platform:=drone', 'use_nav2:=false', 'use_rviz:=false',
            'use_dashboard:=false', 'gazebo_gui:=false', 'scenario:=no_fire',
            'drone_x:=12.0', 'drone_y:=5.0', 'drone_z:=8.0'],
            stdout=log, stderr=subprocess.STDOUT, env=environment, start_new_session=True)
        try:
            until(lambda: client.service_is_ready() and values['fire'] is not None
                  and values['fire'].rendered and len(values['readings']) >= 5)
            for name in ('healthy', 'burning', 'burnt'):
                request = SetFireState.Request()
                request.state = name
                future = client.call_async(request)
                until(future.done, 15)
                assert future.result().accepted, future.result().message
                # Discard samples acquired before the completed geometry change.
                changed = time.monotonic()
                until(lambda: len([r for t, r in values['readings'] if t > changed + 3]) >= 10, 30)
                readings = [r for t, r in values['readings'] if t > changed + 3][-10:]
                assert all(r.thermal_valid and r.co2_valid for r in readings), 'Required channels unavailable'
                result[name] = {
                    'smoke_ppm': sum(r.smoke_ppm for r in readings) / len(readings),
                    'co2_ppm': sum(r.co2_ppm for r in readings) / len(readings),
                    'ir_peak_c': max(r.thermal_peak_c for r in readings),
                }
            assert result['burning']['smoke_ppm'] > result['healthy']['smoke_ppm'] + 3
            assert result['burning']['co2_ppm'] > result['healthy']['co2_ppm'] + 50
            assert result['burning']['ir_peak_c'] > 70, 'Rendered fire did not reach native IR stream'
            assert result['burnt']['smoke_ppm'] < result['burning']['smoke_ppm'] / 2
            assert result['burnt']['ir_peak_c'] < 70, 'Burnt tree retained burning IR geometry'
            result['passed'] = True
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
            (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
