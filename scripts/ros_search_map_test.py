#!/usr/bin/env python3
"""Require a surveyed search detour when the initial upwind view is unknown."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from embertrace_interfaces.msg import EnvironmentalReading, MissionState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', type=Path)
    parser.add_argument('--output', type=Path, default=Path('log/search-map-regression'))
    args = parser.parse_args()
    rclpy.init()
    node = Node('search_map_regression')
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    environment = node.create_publisher(EnvironmentalReading, '/embertrace/active_environment', qos_profile_sensor_data)
    mission = node.create_publisher(MissionState, '/embertrace/mission_state', transient)
    maps = node.create_publisher(OccupancyGrid, '/husky1/map', transient)
    goals = []
    subscription = node.create_subscription(PoseStamped, '/embertrace/navigation_goal', goals.append, 10)
    grid = OccupancyGrid()
    grid.header.frame_id = 'husky1_odom'
    grid.info.width = grid.info.height = 40
    grid.info.resolution = .25
    grid.info.origin.position.x = grid.info.origin.position.y = -5.
    grid.info.origin.orientation.w = 1.
    grid.data = [-1] * 1600
    for row in range(10, 30):
        for col in range(4, 16):
            grid.data[row * 40 + col] = 0

    def pump(seconds, readings=False):
        end = time.monotonic() + seconds
        next_reading = 0.
        while time.monotonic() < end:
            if readings and time.monotonic() >= next_reading:
                msg = EnvironmentalReading()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'husky1_odom'
                msg.sample_pose.header = msg.header
                msg.sample_pose.pose.orientation.w = 1.
                msg.smoke_ppm = 40.
                msg.temperature_c = 35.
                msg.smoke_detected = True
                msg.wind_direction_rad = math.pi
                environment.publish(msg)
                next_reading = time.monotonic() + .05
            rclpy.spin_once(node, timeout_sec=.01)

    def state(phase, epoch):
        msg = MissionState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.active_platform = msg.PLATFORM_ROVER
        msg.state = phase
        msg.mission_epoch = epoch
        msg.motion_allowed = phase == msg.INVESTIGATE
        mission.publish(msg)
        pump(.2)

    root = args.output
    root.mkdir(parents=True, exist_ok=True)
    executable = args.executable or Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/search_planner_node'
    with (root / 'search.log').open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'goal_interval_s:=0.2'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 10
            while environment.get_subscription_count() == 0 and time.monotonic() < deadline:
                pump(.05)
            assert environment.get_subscription_count(), 'Planner not discovered'
            maps.publish(grid)
            pump(.3)
            state(MissionState.INVESTIGATE, 1)
            pump(1.5, True)
            assert goals, 'Initial unknown upwind view left search stuck'
            for goal in goals:
                x, y = goal.pose.position.x, goal.pose.position.y
                assert math.hypot(x, y) >= 1.5, 'Probe stops inside arrival tolerance'
                assert math.hypot(x-2, y) <= 4., 'Detour exceeded its bound'
                assert -3.2 < x < -1.8 and -1.7 < y < 1.7, 'Goal left the surveyed clear patch'
            grid.data = [-1] * 1600
            maps.publish(grid)
            pump(.4)
            state(MissionState.INVESTIGATE, 2)
            goals.clear()
            pump(.8, True)
            assert not goals, 'Unknown map was treated as free'
            grid.data = [100] * 1600
            maps.publish(grid)
            pump(.3)
            state(MissionState.INVESTIGATE, 3)
            goals.clear()
            pump(.8, True)
            assert not goals, 'Occupied map was treated as free'
            print('Search map DDS regression passed: bounded progressing detour; unknown/occupied maps held', flush=True)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
