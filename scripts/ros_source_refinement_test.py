#!/usr/bin/env python3
"""Check immediate alerts, source updates, bounded fallback and wind holds over DDS."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from embertrace_interfaces.msg import EnvironmentalReading, FireAlert, MissionState
from embertrace_interfaces.srv import OperatorCommand
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped


def main():
    output = Path('log/source-refinement-dds')
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('source_refinement_test')
    received = {'state': None, 'alerts': []}
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(MissionState, '/embertrace/mission_state', lambda m: received.update(state=m), transient),
        node.create_subscription(FireAlert, '/embertrace/fire_alert', lambda m: received['alerts'].append(m), transient),
    ]
    readings = node.create_publisher(EnvironmentalReading, '/embertrace/environment', qos_profile_sensor_data)
    wind = node.create_publisher(Vector3Stamped, '/embertrace/wind', qos_profile_sensor_data)
    estimates = node.create_publisher(PoseWithCovarianceStamped, '/embertrace/source_estimate', 10)
    client = node.create_client(OperatorCommand, '/embertrace/operator_command')
    next_sample = 0
    inputs = {}

    def pump(seconds, fire=False, source=False, speed=7, source_age=0, suspected=False):
        nonlocal next_sample
        inputs.update(fire=fire, source=source, speed=speed, source_age=source_age, suspected=suspected)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if time.monotonic() >= next_sample:
                stamp = node.get_clock().now().to_msg()
                ambient = Vector3Stamped()
                ambient.header.stamp = stamp
                ambient.header.frame_id = 'husky1_odom'
                ambient.vector.x = float(speed)
                wind.publish(ambient)
                reading = EnvironmentalReading()
                reading.header = ambient.header
                reading.sample_pose.header = reading.header
                reading.sample_pose.pose.orientation.w = 1.0
                reading.sample_pose.pose.position.x = 1.0
                reading.sample_pose.pose.position.y = 2.0
                reading.smoke_ppm = 55.0 if fire else (20.0 if suspected else 1.0)
                reading.temperature_c = 80.0 if fire else 22.0
                reading.co2_ppm = 1500.0 if fire else 420.0
                reading.fused_confidence = .95 if fire else (.5 if suspected else 0.0)
                reading.smoke_detected = reading.co2_detected = fire or suspected
                reading.temperature_anomaly = fire
                reading.thermal_valid = reading.co2_valid = True
                reading.thermal_peak_c = 100.0 if fire else 22.0
                readings.publish(reading)
                if source:
                    fit = PoseWithCovarianceStamped()
                    fit.header.frame_id = reading.header.frame_id
                    fit.header.stamp = (node.get_clock().now() - Duration(seconds=source_age)).to_msg()
                    fit.pose.pose.orientation.w = 1.0
                    # Controlled estimator output, unrelated to simulator truth.
                    fit.pose.pose.position.x = 6.0
                    fit.pose.pose.position.y = 9.0
                    fit.pose.covariance[0] = fit.pose.covariance[7] = .25
                    estimates.publish(fit)
                next_sample = time.monotonic() + .2
            rclpy.spin_once(node, timeout_sec=.02)

    def until(predicate, timeout=8, **kwargs):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pump(.1, **kwargs)
            if predicate(): return
        raise AssertionError(f'Refinement deadline: {received}')

    def command(text, expected=True):
        request = OperatorCommand.Request()
        request.command = text
        future = client.call_async(request)
        deadline = time.monotonic() + 3
        # Sensors continue running while the operator awaits a service response.
        while not future.done() and time.monotonic() < deadline:
            pump(.05, **inputs)
        assert future.done() and future.result().accepted == expected, text
        print(f'{text}: {future.result().message}', flush=True)

    def confirmed(after=0):
        return [a for a in received['alerts'] if a.confirmed and a.alert_id > after]

    executable = Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/mission_manager_node'
    with (output / 'manager.log').open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args',
            '-p', 'drone_enabled:=false', '-p', 'localise_before_hold:=true',
            '-p', 'localisation_timeout_s:=3.0', '-p', 'search_timeout_s:=2.5', '-p', 'wind_dwell_s:=0.3'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            until(lambda: client.service_is_ready() and received['state'] is not None)
            pump(1)
            command('START')
            until(lambda: len([a for a in received['alerts'] if not a.confirmed]) >= 3,
                  timeout=12, suspected=True)
            warnings = [a for a in received['alerts'] if not a.confirmed]
            assert len({a.alert_id for a in warnings}) == 1, 'Investigation retries created new anomaly IDs'
            until(lambda: bool(confirmed()) and received['state'].state == MissionState.TRACK, fire=True)
            first = confirmed()[0]
            assert first.alert_id == warnings[0].alert_id, 'Confirmation did not promote the existing investigation'
            assert not first.source_resolved and received['state'].motion_allowed
            until(lambda: received['state'].state == MissionState.ALERT and
                  any(a.source_resolved for a in confirmed()), fire=True, source=True)
            update = confirmed()[-1]
            assert update.alert_id == first.alert_id, 'Source update created a second fire'
            assert update.estimated_source.pose.position.x == 6
            assert not received['state'].motion_allowed
            command('RESET')
            pump(.4)
            command('START')
            until(lambda: bool(confirmed(first.alert_id)) and received['state'].state == MissionState.TRACK, fire=True, source=True, source_age=2)
            second = confirmed(first.alert_id)[0]
            assert not second.source_resolved, 'Previous mission source estimate survived reset'
            # Weaker survey readings must not erase an already confirmed incident.
            until(lambda: received['state'].state == MissionState.ALERT, timeout=6)
            fallback = confirmed()[-1]
            assert fallback.alert_id == second.alert_id and not fallback.source_resolved
            assert fallback.confidence >= .74, 'Fallback lost original confirmation evidence'
            assert fallback.estimated_source.pose.position.x == 1
            command('RESET')
            pump(.4)
            command('START')
            until(lambda: bool(confirmed(second.alert_id)) and received['state'].state == MissionState.TRACK, fire=True)
            pump(.4, fire=True, speed=13)
            pump(.4, fire=True, source=True, speed=13)
            assert received['state'].state == MissionState.PAUSED and not received['state'].motion_allowed
            command('RESUME', expected=False)
            print('Source DDS passed: immediate alert, stable ID, fresh source, bounded fallback and unsafe-wind hold')
        finally:
            if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
