#!/usr/bin/env python3
"""Exercise the live wind/battery simulator and mission manager over ROS transport."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from embertrace_interfaces.msg import EnvironmentalReading, MissionState
from embertrace_interfaces.srv import OperatorCommand
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    rclpy.init()
    node = Node('mission_inputs_integration')
    received = {'state': None, 'battery': None}
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(MissionState, '/embertrace/mission_state',
                                 lambda m: received.update(state=m), transient),
        node.create_subscription(BatteryState, '/drone1/battery',
                                 lambda m: received.update(battery=m.percentage), 10),
    ]
    readings = [node.create_publisher(EnvironmentalReading, root+'/environment', qos_profile_sensor_data)
                for root in ('/embertrace', '/drone1')]
    arrived = node.create_publisher(String, '/drone1/mission_navigation_status', 10)
    operator = node.create_client(OperatorCommand, '/embertrace/operator_command')
    parameters = node.create_client(SetParameters, '/mission_simulator/set_parameters')
    next_sample = 0

    def pump(seconds):
        nonlocal next_sample
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            if time.monotonic()>=next_sample:
                reading = EnvironmentalReading(smoke_ppm=1., temperature_c=22., co2_ppm=420.,
                                               co2_valid=True, thermal_valid=True, thermal_peak_c=22.)
                reading.header.stamp = node.get_clock().now().to_msg()
                reading.header.frame_id = 'husky1_odom'
                reading.sample_pose.header = reading.header
                reading.sample_pose.pose.orientation.w = 1.
                for publisher in readings: publisher.publish(reading)
                next_sample = time.monotonic()+.1
            rclpy.spin_once(node, timeout_sec=.01)

    def until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            pump(.05)
            if predicate(): return
        raise AssertionError(f'Mission input deadline: {received}')

    def request(client, message):
        future = client.call_async(message)
        until(future.done, timeout=3)
        return future.result()

    def command(text, accepted=True):
        response = request(operator, OperatorCommand.Request(command=text))
        assert response.accepted == accepted, (text, response.message)

    def setting(name, value, accepted=True):
        parameter = Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
        response = request(parameters, SetParameters.Request(parameters=[parameter]))
        assert len(response.results)==1 and response.results[0].successful == accepted, (name, response)

    def phase(value):
        return received['state'] is not None and received['state'].state == value

    output = Path('log/mission-inputs-dds')
    output.mkdir(parents=True, exist_ok=True)
    root = Path(get_package_prefix('embertrace_core'))/'lib/embertrace_core'
    processes, handles = [], []
    try:
        for name, options in (
            ('mission_manager_node', ['--ros-args', '-p', 'wind_dwell_s:=0.3']),
            ('mission_simulator.py', []),
        ):
            handle = (output/(name+'.log')).open('w')
            handles.append(handle)
            processes.append(subprocess.Popen([str(root/name), *options], stdout=handle,
                             stderr=subprocess.STDOUT, start_new_session=True))
        until(lambda: operator.service_is_ready() and parameters.service_is_ready() and
              received['state'] is not None and received['state'].preferred_platform == MissionState.PLATFORM_DRONE)
        command('START')
        until(lambda: phase(MissionState.PATROL))
        assert received['state'].active_platform == MissionState.PLATFORM_DRONE
        setting('initial_battery_fraction', .19)
        until(lambda: phase(MissionState.RETURN_TO_BASE))
        assert 'battery' in received['state'].detail.lower(), received['state'].detail
        # Arrival is a controlled navigation fixture; no operator RETURN is sent.
        arrived.publish(String(data='RETURNED'))
        until(lambda: phase(MissionState.IDLE))
        setting('initial_battery_fraction', 1.)
        until(lambda: received['battery'] is not None and received['battery']>.99)
        command('START')
        setting('active_drain_per_second', .8)
        until(lambda: phase(MissionState.RETURN_TO_BASE))
        assert received['battery']<.2, received['battery']
        command('RESET')
        setting('active_drain_per_second', .0006)
        setting('initial_battery_fraction', .99)
        setting('wind_speed_mps', 7.)
        until(lambda: received['state'].preferred_platform == MissionState.PLATFORM_ROVER)
        command('START')
        until(lambda: phase(MissionState.PATROL))
        assert received['state'].active_platform == MissionState.PLATFORM_ROVER
        setting('wind_speed_mps', 13.)
        until(lambda: phase(MissionState.PAUSED))
        assert not received['state'].motion_allowed
        command('RESUME', accepted=False)
        setting('wind_speed_mps', -1., accepted=False)
        setting('initial_battery_fraction', 1.1, accepted=False)
        setting('wind_speed_mps', 7.)
        until(lambda: received['state'].preferred_platform == MissionState.PLATFORM_ROVER)
        command('RESUME')
        until(lambda: phase(MissionState.PATROL))
        stop(processes[1])
        until(lambda: phase(MissionState.PAUSED))
        assert not received['state'].motion_allowed
        command('RESUME', accepted=False)
        print('Mission inputs passed: live parameters, battery threshold/depletion return, wind selection/holds, rejected settings and input loss')
    finally:
        for process in reversed(processes): stop(process)
        for handle in handles: handle.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
