#!/usr/bin/env python3
"""Assert live wind gating and monotonic watchdogs, including a frozen ROS clock."""
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
from embertrace_interfaces.msg import MissionState, PerceptionStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool


def main():
    output = Path('log/motion-integration'); output.mkdir(parents=True, exist_ok=True)
    rclpy.init(); node = Node('motion_integration_test')
    command = node.create_publisher(Twist, '/husky1/autonomy_cmd_vel', 10)
    mission = node.create_publisher(MissionState, '/embertrace/mission_state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clock = node.create_publisher(Clock, '/clock', 10)
    odom = node.create_publisher(Odometry, '/drone1/odometry', qos_profile_sensor_data)
    health = [node.create_publisher(Bool, topic, 10) for topic in
              ('/embertrace/sensors_ready', '/drone1/sensors_ready')]
    status = [node.create_publisher(PerceptionStatus, topic, 10) for topic in
              ('/embertrace/perception_status', '/drone1/perception_status')]
    velocities = {}
    subscriptions = [node.create_subscription(Twist, f'/{robot}/cmd_vel',
        lambda msg, robot=robot: velocities.update({robot: (time.monotonic(), msg)}), 10)
        for robot in ('husky1', 'drone1')]
    processes, handles = [], []
    simulated = 100.
    def pump(seconds, wind=PerceptionStatus.WIND_DRONE_SAFE, allowed=True, streams=True, frozen=False):
        nonlocal simulated
        deadline = time.monotonic() + seconds; next_sample = 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_sample:
                if not frozen:
                    simulated += .05
                value = Clock(); value.clock.sec = int(simulated)
                value.clock.nanosec = int((simulated % 1) * 1e9); clock.publish(value)
                if streams:
                    state = MissionState(); state.header.stamp = value.clock
                    state.state = state.PATROL if allowed else state.EMERGENCY_STOP
                    state.motion_allowed = allowed; mission.publish(state)
                    reading = PerceptionStatus(); reading.header.stamp = value.clock
                    reading.environment_fresh = True; reading.wind_condition = wind
                    for publisher in status: publisher.publish(reading)
                    for publisher in health: publisher.publish(Bool(data=True))
                    pose = Odometry(); pose.header.stamp = value.clock; pose.header.frame_id = 'husky1_odom'
                    pose.pose.pose.position.x = -14.; pose.pose.pose.position.y = -12.
                    pose.pose.pose.position.z = 8.; pose.pose.pose.orientation.w = 1.; odom.publish(pose)
                    velocity = Twist(); velocity.linear.x = .4; command.publish(velocity)
                next_sample = time.monotonic() + .05
            rclpy.spin_once(node, timeout_sec=.01)
    def speed(robot):
        assert robot in velocities and time.monotonic() - velocities[robot][0] < .8, f'No {robot} heartbeat'
        twist = velocities[robot][1]
        return math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
    try:
        for executable in ('velocity_guard_node', 'drone_scout_node'):
            handle = (output / f'{executable}.log').open('w'); handles.append(handle)
            path = Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core' / executable
            processes.append(subprocess.Popen([str(path), '--ros-args', '-p', 'use_sim_time:=true'],
                stdout=handle, stderr=subprocess.STDOUT, start_new_session=True))
        pump(2.5)
        assert speed('husky1') > .1 and speed('drone1') > .1, 'Safe wind did not permit motion'
        pump(.5, PerceptionStatus.WIND_ROVER_ONLY)
        assert speed('husky1') > .1 and speed('drone1') < 1e-6, 'Rover-only wind policy failed'
        for wind in (PerceptionStatus.WIND_UNSAFE, PerceptionStatus.WIND_UNKNOWN):
            pump(.4, wind)
            assert all(speed(r) < 1e-6 for r in velocities), 'Unknown/unsafe wind permitted motion'
        pump(.4); assert speed('husky1') > .1 and speed('drone1') > .1
        pump(.4, allowed=False)
        assert all(speed(r) < 1e-6 for r in velocities), 'Motion-disallowed state failed'
        pump(.4)
        pump(2., streams=False, frozen=True)
        assert all(speed(r) < 1e-6 for r in velocities), 'Frozen ROS time defeated receipt watchdogs'
        print('Motion DDS integration passed: vehicle wind limits, state inhibition and frozen-clock heartbeat expiry', flush=True)
    finally:
        for process in processes:
            if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
        for handle in handles: handle.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
