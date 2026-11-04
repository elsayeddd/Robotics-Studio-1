#!/usr/bin/env python3
"""Readiness metadata must identify its environmental acquisition, not publication."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from embertrace_interfaces.msg import EnvironmentalReading, PerceptionStatus


def main():
    rclpy.init()
    node = Node('status_timestamp_test')
    sent, matched = set(), []
    def status(message):
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        if message.environment_fresh and key in sent and message.header.frame_id == 'husky1_odom':
            matched.append(message)
    subscription = node.create_subscription(PerceptionStatus, '/embertrace/perception_status', status, 10)
    publisher = node.create_publisher(EnvironmentalReading, '/embertrace/raw_environment', qos_profile_sensor_data)
    executable = Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/perception_node'
    output = Path('log/status-timestamp')
    output.mkdir(parents=True, exist_ok=True)
    with (output / 'perception.log').open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'warmup_samples:=1',
            '-p', 'require_co2:=true', '-p', 'require_thermal:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline, next_sample = time.monotonic() + 8, 0
            while time.monotonic() < deadline and not matched:
                if publisher.get_subscription_count() and time.monotonic() >= next_sample:
                    reading = EnvironmentalReading()
                    reading.header.stamp = (node.get_clock().now() - Duration(seconds=.25)).to_msg()
                    reading.header.frame_id = 'husky1_odom'
                    reading.sample_pose.header = reading.header
                    reading.sample_pose.pose.orientation.w = 1.0
                    reading.smoke_ppm = 1.0
                    reading.temperature_c = 22.0
                    reading.co2_ppm = 420.0
                    reading.co2_valid = True
                    sent.add((reading.header.stamp.sec, reading.header.stamp.nanosec))
                    publisher.publish(reading)
                    next_sample = time.monotonic() + .2
                rclpy.spin_once(node, timeout_sec=.02)
            assert matched, 'Status publication time obscured its originating acquisition/frame'
            print('Perception status acquisition timestamp/frame passed')
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
