#!/usr/bin/env python3
"""Exercise mission transitions, search completion and coordinated return over DDS."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from embertrace_interfaces.msg import EnvironmentalReading, MissionState
from embertrace_interfaces.srv import OperatorCommand
from std_msgs.msg import Bool, Empty, String


def main():
    output = Path('log/mission-integration'); output.mkdir(parents=True, exist_ok=True)
    executable = Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/mission_manager_node'
    rclpy.init(); node = Node('mission_integration_test')
    states = []
    subscription = node.create_subscription(MissionState, '/embertrace/mission_state',
        lambda msg: states.append(msg.state_name), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    environment = node.create_publisher(EnvironmentalReading, '/embertrace/environment', qos_profile_sensor_data)
    search = node.create_publisher(String, '/embertrace/search_status', 10)
    rover_home = node.create_publisher(Empty, '/embertrace/return_complete', 10)
    drone_home = node.create_publisher(Bool, '/drone1/return_complete', 10)
    client = node.create_client(OperatorCommand, '/embertrace/operator_command')
    with (output / 'mission.log').open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'require_scout_return:=true',
            '-p', 'reading_timeout_s:=0.8'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            assert client.wait_for_service(timeout_sec=15)
            def pump(seconds, confidence=0., smoke=False, heat=False, readings=True):
                deadline = time.monotonic() + seconds
                next_sample = 0
                while time.monotonic() < deadline:
                    if readings and time.monotonic() >= next_sample:
                        msg = EnvironmentalReading(); msg.header.stamp = node.get_clock().now().to_msg()
                        msg.header.frame_id = 'husky1_odom'; msg.sample_pose.header = msg.header
                        msg.sample_pose.pose.orientation.w = 1.
                        msg.fused_confidence = float(confidence)
                        msg.smoke_detected = smoke; msg.temperature_anomaly = heat
                        msg.visual_confidence = 0.
                        environment.publish(msg); next_sample = time.monotonic() + .05
                    rclpy.spin_once(node, timeout_sec=.01)
            def command(text, accepted=True):
                request = OperatorCommand.Request(); request.command = text
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().accepted == accepted, text
                pump(.12)
            def state(expected):
                assert states and states[-1] == expected, f'Expected {expected}, got {states[-8:]}'
            pump(.4); state('IDLE')
            command('UNKNOWN', False); command('RESUME', False)
            command('START'); state('PATROL')
            command('PAUSE'); command('PAUSE'); state('PAUSED')
            command('RESUME'); state('PATROL')
            pump(.5, .3, True); state('INVESTIGATE')
            search.publish(String(data='TIMED_OUT')); pump(.2, .3, True); state('PATROL')
            pump(.5, .4, True); state('PATROL')  # Do not repeatedly investigate the same dismissed anomaly.
            pump(.2); pump(.5, .5, True); state('TRACK')
            pump(.2, .05); state('RECOVER')
            pump(.2, .5, True); state('TRACK')
            pump(.5, .9, True, True); state('ALERT')
            command('RETURN'); state('RETURN_TO_BASE')
            rover_home.publish(Empty()); pump(.3); state('RETURN_TO_BASE')
            drone_home.publish(Bool(data=True)); pump(.3); state('IDLE')
            command('START'); command('E_STOP'); state('EMERGENCY_STOP')
            for text in ('START', 'RESUME', 'RETURN', 'PAUSE'):
                command(text, False); state('EMERGENCY_STOP')
            command('E_STOP_RELEASE'); state('IDLE')
            command('START'); pump(1.3, readings=False); state('PAUSED')
            print('Mission DDS integration passed: patrol, pause/resume, search recovery, alert, both returns, E-stop, dropout', flush=True)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
