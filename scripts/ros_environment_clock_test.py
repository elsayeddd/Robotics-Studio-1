#!/usr/bin/env python3
"""A frozen clock must not generate repeated environmental/truth acquisitions."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from embertrace_interfaces.msg import EnvironmentalReading, TrialTruth
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def main():
    rclpy.init()
    node = Node('environment_clock_test')
    raw, truth = [], []
    key = lambda m: (m.header.stamp.sec, m.header.stamp.nanosec)
    subscriptions = [
        node.create_subscription(EnvironmentalReading, '/embertrace/raw_environment', lambda m: raw.append(key(m)), qos_profile_sensor_data),
        node.create_subscription(TrialTruth, '/embertrace/validation/truth', lambda m: truth.append(key(m)), 10),
    ]
    clock = node.create_publisher(Clock, '/clock', 10)
    odometry = node.create_publisher(Odometry, '/husky1/odometry', qos_profile_sensor_data)
    def pump(seconds, stamp):
        deadline, next_sample = time.monotonic() + seconds, 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_sample:
                tick = Clock()
                tick.clock.sec, tick.clock.nanosec = stamp
                clock.publish(tick)
                pose = Odometry()
                pose.header.stamp = tick.clock
                pose.header.frame_id = 'husky1_odom'
                pose.pose.pose.orientation.w = 1.0
                odometry.publish(pose)
                next_sample = time.monotonic() + .05
            rclpy.spin_once(node, timeout_sec=.01)
    output = Path('log/environment-clock')
    output.mkdir(parents=True, exist_ok=True)
    executable = Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/environment_simulator_node'
    with (output / 'simulator.log').open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'use_sim_time:=true',
            '-p', 'require_wind:=false', '-p', 'scenario:=no_fire', '-p', 'publish_rate_hz:=20.0'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            first = (100, 0)
            deadline = time.monotonic() + 8
            while (first not in raw or first not in truth) and time.monotonic() < deadline:
                pump(.1, first)
            assert first in raw and first in truth, 'Initial acquisition missing'
            pump(.6, first)
            assert raw.count(first) == truth.count(first) == 1, 'Frozen clock emitted duplicate acquisitions'
            second = (100, 200000000)
            pump(.6, second)
            assert raw.count(second) == truth.count(second) == 1, 'Clock advance was not sampled exactly once'
            reset = (1, 0)
            pump(.7, reset)
            assert raw.count(reset) == truth.count(reset) == 1, 'New simulation epoch did not restart sampling'
            print('Environment clock passed: frozen acquisition suppression, advance and epoch reset')
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
