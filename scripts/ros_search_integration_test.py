#!/usr/bin/env python3
"""Check the live search node approaches a fitted fire from safe downwind space."""
import math
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String


def main():
    output = Path('log/search-integration'); output.mkdir(parents=True, exist_ok=True)
    rclpy.init(); node = Node('search_integration_test')
    environment = node.create_publisher(EnvironmentalReading, '/embertrace/active_environment', qos_profile_sensor_data)
    mission = node.create_publisher(MissionState, '/embertrace/mission_state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    received = {'goals': [], 'estimate': None, 'status': []}
    subscriptions = [
        node.create_subscription(PoseStamped, '/drone1/navigation_goal', lambda m: received['goals'].append(m), 10),
        node.create_subscription(PoseWithCovarianceStamped, '/embertrace/source_estimate', lambda m: received.update(estimate=m), 10),
        node.create_subscription(String, '/embertrace/search_status', lambda m: received['status'].append(m.data), 10),
    ]
    wind = -3 * math.pi / 4
    def reading(x, y):
        dx, dy = x - 12., y - 7.
        distance = math.hypot(dx, dy)
        down = dx * math.cos(wind) + dy * math.sin(wind)
        cross = -dx * math.sin(wind) + dy * math.cos(wind)
        plume = 120 * math.exp(-down / 14) * math.exp(-.5 * (cross / (1.6 + .05 * down))**2) if down >= 0 else 3.6 * math.exp(-distance / 1.5)
        msg = EnvironmentalReading(); msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'husky1_odom'; msg.sample_pose.header = msg.header
        msg.sample_pose.pose.position.x = float(x); msg.sample_pose.pose.position.y = float(y)
        msg.sample_pose.pose.position.z = 8.; msg.sample_pose.pose.orientation.w = 1.
        msg.smoke_ppm = 1 + plume * math.exp(-1)
        msg.temperature_c = 22 + 88 * math.exp(-distance / 7) * math.exp(-1)
        msg.wind_direction_rad = wind; msg.wind_speed_mps = 2.5
        msg.smoke_detected = msg.smoke_ppm >= 12; msg.temperature_anomaly = msg.temperature_c >= 32
        return msg
    def spin(seconds, pose=None):
        end = time.monotonic() + seconds; next_reading = 0
        while time.monotonic() < end:
            if pose and time.monotonic() >= next_reading:
                environment.publish(reading(*pose)); next_reading = time.monotonic() + .05
            rclpy.spin_once(node, timeout_sec=.01)
    def state(value):
        msg = MissionState(); msg.header.stamp = node.get_clock().now().to_msg()
        msg.active_platform = msg.PLATFORM_DRONE; msg.mission_epoch = 1
        msg.state = value; msg.motion_allowed = value in (msg.PATROL, msg.INVESTIGATE)
        mission.publish(msg); spin(.1)
    executable = Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/search_planner_node'
    with (output / 'search.log').open('w') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'require_map:=false',
            '-p', 'goal_interval_s:=0.2', '-p', 'estimate_interval_s:=0.2',
            '-p', 'signal_loss_timeout_s:=1.0', '-p', 'search_timeout_s:=3.0'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            end = time.monotonic() + 10
            while environment.get_subscription_count() == 0 and time.monotonic() < end: spin(.05)
            assert environment.get_subscription_count(), 'Search subscription unavailable'
            state(MissionState.PATROL)
            for x in (8., 10., 12., 14.):
                for y in (3., 5., 7., 9.):
                    environment.publish(reading(x, y)); spin(.07)
            state(MissionState.INVESTIGATE)
            end = time.monotonic() + 3
            while (received['estimate'] is None or not received['goals']) and time.monotonic() < end:
                spin(.1, (12.2, 6.8))
            assert received['estimate'] is not None and received['goals'], 'Search did not fit and publish'
            estimate = received['estimate'].pose.pose.position
            goal = received['goals'][-1].pose.position
            assert math.hypot(estimate.x - 12, estimate.y - 7) < .3, 'Controlled local samples did not fit the source'
            downwind = (goal.x - estimate.x) * math.cos(wind) + (goal.y - estimate.y) * math.sin(wind)
            assert downwind > 1., f'Inspection goal is not downwind: {downwind}'
            assert abs(math.hypot(goal.x - estimate.x, goal.y - estimate.y) - 1.2) < .2, 'Source stand-off lost'
            assert reading(goal.x, goal.y).smoke_ppm > 30, 'Goal remained in the weak upwind halo'
            state(MissionState.PAUSED); received['goals'].clear(); spin(.5, (12.2, 6.8))
            assert not received['goals'], 'Paused search continued issuing goals'
            state(MissionState.PATROL); spin(.1, (10., 5.)); spin(1.3)
            received['status'].clear(); state(MissionState.INVESTIGATE); spin(1.1, (12.2, 6.8))
            assert 'SIGNAL_LOST' not in received['status'], 'Previous investigation signal age aborted the new search'
            print('Search DDS integration passed: fitted source, downwind stand-off, pause and fresh investigation timer', flush=True)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
