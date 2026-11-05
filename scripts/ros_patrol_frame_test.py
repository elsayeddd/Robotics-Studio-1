#!/usr/bin/env python3
"""Check route-frame precedence and home capture on a configured odometry topic."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from embertrace_interfaces.msg import MissionState
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def main():
    rclpy.init()
    node = Node('patrol_frame_regression')
    mission = node.create_publisher(MissionState, '/embertrace/mission_state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odometry = node.create_publisher(Odometry, '/test_rover/odometry', qos_profile_sensor_data)
    transforms = TransformBroadcaster(node)
    goals = []
    subscription = node.create_subscription(PoseStamped, '/embertrace/navigation_goal', goals.append, 10)
    pose = [1., 2.]

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = Odometry()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'test_odom'
            msg.child_frame_id = 'test_base'
            msg.pose.pose.position.x, msg.pose.pose.position.y = pose
            msg.pose.pose.orientation.w = 1.
            odometry.publish(msg)
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = msg.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = pose
            tf.transform.rotation.w = 1.
            transforms.sendTransform(tf)
            rclpy.spin_once(node, timeout_sec=.02)

    def state(phase):
        msg = MissionState()
        msg.state = phase
        msg.active_platform = msg.PLATFORM_ROVER
        msg.motion_allowed = True
        msg.mission_epoch = 1
        mission.publish(msg)
        pump(.6)

    root = Path('log/patrol-frame-regression')
    root.mkdir(parents=True, exist_ok=True)
    executable = Path(get_package_prefix('embertrace_core')) / 'lib/embertrace_core/patrol_manager_node'
    with tempfile.TemporaryDirectory(prefix='embertrace_patrol_frame_') as directory, (root / 'patrol.log').open('w') as log:
        route = Path(directory) / 'route.yaml'
        route.write_text('frame_id: test_odom\nwaypoints:\n'
            '  - {name: ranger_base, x: 0.0, y: 0.0, yaw: 0.0}\n'
            '  - {name: next, x: 5.0, y: 2.0, yaw: 0.0}\n')
        process = subprocess.Popen([str(executable), '--ros-args',
            '-p', 'waypoints_file:='+str(route), '-p', 'map_frame:=test_map',
            '-p', 'odom_frame:=test_odom', '-p', 'base_frame:=test_base',
            '-p', 'odometry_topic:=/test_rover/odometry'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 8
            while not odometry.get_subscription_count() and time.monotonic() < deadline:
                pump(.1)
            assert odometry.get_subscription_count(), 'Configured odometry topic was ignored'
            pump(.4)
            state(MissionState.PATROL)
            assert goals and all(g.header.frame_id == 'test_odom' for g in goals), 'Map alias replaced the route frame'
            pose[:] = [3., 2.]
            pump(.3)
            goals.clear()
            state(MissionState.RETURN_TO_BASE)
            assert goals, 'No return goal published'
            home = goals[-1].pose.position
            assert abs(home.x-1.) < 1e-6 and abs(home.y-2.) < 1e-6, 'Return did not use the captured odometry home'
            print('Patrol DDS regression passed: route frame preserved and configured odometry home used', flush=True)
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
